"""Arrow/Parquet file writer for time-series data logging."""

import time
from pathlib import Path
from typing import List, Optional

import pyarrow as pa
import pyarrow.parquet as pq

from .models import (
    ConnectedMessage,
    GrindFinishedMessage,
    ScaleSetting,
    StateChangeMessage,
    TargetWeightChangedMessage,
    WeightMessage,
    WsMessage,
)


# Arrow schema for logged data
SCHEMA = pa.schema([
    ("timestamp_ms", pa.uint64()),           # Device timestamp
    ("received_at", pa.float64()),           # Local Unix timestamp when received
    ("message_type", pa.string()),           # "Connected" | "StateChange" | "WeightReading" | "TargetWeightChanged" | "GrindFinished"
    ("state", pa.string()),                  # UserEvent as string
    ("weight", pa.float32()),                # Current weight (nullable)
    ("coffee_weight", pa.float32()),         # Coffee weight only (nullable)
    ("scale_offset", pa.float32()),          # Scale calibration (nullable)
    ("scale_inv_variance", pa.float32()),    # Scale calibration (nullable)
    ("scale_factor", pa.float32()),          # Scale calibration (nullable)
    ("target_weight", pa.float32()),         # Target coffee weight (nullable)
    ("filtered_weight", pa.float32()),       # GP estimate of the coffee weight while grinding (nullable)
    ("eta_median", pa.float32()),            # Seconds until the grinder stops, median (nullable)
    ("eta_lo", pa.float32()),                # ... 10 % quantile (nullable)
    ("eta_hi", pa.float32()),                # ... 90 % quantile (nullable)
    ("lead_time", pa.float32()),             # Lead time in seconds (nullable)
    ("stop_reason", pa.string()),            # "Prediction" | "RawWeight" | "Timeout" (GrindFinished only)
    ("stop_weight", pa.float32()),           # Coffee weight when the grinder stopped (GrindFinished only)
    ("settled_weight", pa.float32()),        # Settled coffee weight (GrindFinished only, nullable)
    ("lead_time_observed", pa.float32()),    # Lead time observed in the grind (GrindFinished only, nullable)
])


class ArrowWriter:
    """Batched writer for Arrow/Parquet files with ordered timestamp access."""

    def __init__(self, output_path: str, batch_size: int = 100):
        """
        Initialize the Arrow writer.

        Args:
            output_path: Path to output Parquet file
            batch_size: Number of records to buffer before writing
        """
        self.output_path = Path(output_path)
        self.batch_size = batch_size
        self.batch: List[dict] = []
        self.writer: Optional[pq.ParquetWriter] = None
        # TargetWeightChanged carries no device timestamp; reuse the last one seen.
        self.last_timestamp_ms = 0

    def add_message(self, message: WsMessage) -> None:
        """
        Add a WebSocket message to the batch.

        Args:
            message: Parsed WebSocket message
        """
        received_at = time.time()

        if isinstance(message, (ConnectedMessage, StateChangeMessage)):
            message_type = (
                "Connected" if isinstance(message, ConnectedMessage) else "StateChange"
            )
            self._add_record(
                timestamp_ms=message.timestamp_ms,
                received_at=received_at,
                message_type=message_type,
                state=message.state.name,
                scale_setting=message.scale_setting,
                target_weight=message.target_weight,
                lead_time=message.lead_time,
            )

        elif isinstance(message, WeightMessage):
            reading = message.reading
            self._add_record(
                timestamp_ms=reading.timestamp_ms,
                received_at=received_at,
                message_type="WeightReading",
                state=reading.state.name,
                weight=reading.weight,
                coffee_weight=reading.coffee_weight,
                filtered_weight=reading.filtered_weight,
                eta_median=reading.eta.median if reading.eta else None,
                eta_lo=reading.eta.lo if reading.eta else None,
                eta_hi=reading.eta.hi if reading.eta else None,
            )

        elif isinstance(message, TargetWeightChangedMessage):
            self._add_record(
                timestamp_ms=self.last_timestamp_ms,
                received_at=received_at,
                message_type="TargetWeightChanged",
                state=None,
                target_weight=message.target_weight,
            )

        elif isinstance(message, GrindFinishedMessage):
            # Carries no device timestamp; reuse the last one seen.
            self._add_record(
                timestamp_ms=self.last_timestamp_ms,
                received_at=received_at,
                message_type="GrindFinished",
                state=None,
                stop_reason=message.stop_reason.name,
                stop_weight=message.stop_weight,
                settled_weight=message.settled_weight,
                lead_time_observed=message.lead_time_observed,
                lead_time=message.lead_time,
            )

        # Flush if batch is full
        if len(self.batch) >= self.batch_size:
            self.flush()

    def _add_record(
        self,
        timestamp_ms: int,
        received_at: float,
        message_type: str,
        state: Optional[str],
        scale_setting: Optional[ScaleSetting] = None,
        **fields,
    ) -> None:
        """Add a single record to the batch; columns not given are null."""
        unknown = set(fields) - set(SCHEMA.names)
        assert not unknown, f"unknown columns {unknown}"
        self.last_timestamp_ms = max(self.last_timestamp_ms, timestamp_ms)
        record = dict.fromkeys(SCHEMA.names)
        record.update(
            timestamp_ms=timestamp_ms,
            received_at=received_at,
            message_type=message_type,
            state=state,
            **fields,
        )
        if scale_setting:
            record.update(
                scale_offset=scale_setting.offset,
                scale_inv_variance=scale_setting.inv_variance,
                scale_factor=scale_setting.factor,
            )
        self.batch.append(record)

    def flush(self) -> None:
        """Write batched records to Parquet file."""
        if not self.batch:
            return

        # Sort by timestamp_ms to ensure ordering
        self.batch.sort(key=lambda r: r["timestamp_ms"])

        # Convert to Arrow table
        table = pa.Table.from_pylist(self.batch, schema=SCHEMA)

        # Initialize writer if needed
        if self.writer is None:
            if self.output_path.exists():
                # Append mode: read existing file, concatenate, and rewrite
                # Files from older logger versions may lack newer columns; those
                # get filled with nulls.
                existing_table = pq.read_table(self.output_path)
                table = pa.concat_tables(
                    [existing_table, table], promote_options="default"
                ).select(SCHEMA.names).cast(SCHEMA)
                # Re-sort entire dataset
                indices = pa.compute.sort_indices(table, sort_keys=[("timestamp_ms", "ascending")])
                table = pa.compute.take(table, indices)
                self.output_path.unlink()  # Remove old file

            self.writer = pq.ParquetWriter(
                self.output_path,
                schema=SCHEMA,
                compression="snappy",
            )

        # Write the batch
        self.writer.write_table(table)
        self.batch.clear()
        print(f"Flushed {len(table)} records to {self.output_path}")

    def close(self) -> None:
        """Flush remaining data and close the writer."""
        self.flush()
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        print(f"Closed writer. Data saved to {self.output_path}")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures data is flushed."""
        self.close()

"""Arrow/Parquet file writer for time-series data logging."""

import time
from pathlib import Path
from typing import List, Optional

import pyarrow as pa
import pyarrow.parquet as pq

from .models import (
    ConnectedMessage,
    StateChangeMessage,
    WeightBatchMessage,
    WeightReading,
    WsMessage,
)


# Arrow schema for logged data
SCHEMA = pa.schema([
    ("timestamp_ms", pa.uint64()),           # Device timestamp
    ("received_at", pa.float64()),           # Local Unix timestamp when received
    ("message_type", pa.string()),           # "Connected" | "StateChange" | "WeightReading"
    ("state", pa.string()),                  # UserEvent as string
    ("weight", pa.float32()),                # Current weight (nullable)
    ("coffee_weight", pa.float32()),         # Coffee weight only (nullable)
    ("scale_offset", pa.float32()),          # Scale calibration (nullable)
    ("scale_inv_variance", pa.float32()),    # Scale calibration (nullable)
    ("scale_factor", pa.float32()),          # Scale calibration (nullable)
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

    def add_message(self, message: WsMessage) -> None:
        """
        Add a WebSocket message to the batch.

        Args:
            message: Parsed WebSocket message (Connected, StateChange, or WeightBatch)
        """
        received_at = time.time()

        if isinstance(message, ConnectedMessage):
            self._add_record(
                timestamp_ms=message.timestamp_ms,
                received_at=received_at,
                message_type="Connected",
                state=message.state.name,
                weight=None,
                coffee_weight=None,
                scale_offset=message.scale_setting.offset,
                scale_inv_variance=message.scale_setting.inv_variance,
                scale_factor=message.scale_setting.factor,
            )

        elif isinstance(message, StateChangeMessage):
            self._add_record(
                timestamp_ms=message.timestamp_ms,
                received_at=received_at,
                message_type="StateChange",
                state=message.state.name,
                weight=None,
                coffee_weight=None,
                scale_offset=None,
                scale_inv_variance=None,
                scale_factor=None,
            )

        elif isinstance(message, WeightBatchMessage):
            for reading in message.readings:
                self._add_record(
                    timestamp_ms=reading.timestamp_ms,
                    received_at=received_at,
                    message_type="WeightReading",
                    state=reading.state.name,
                    weight=reading.weight,
                    coffee_weight=reading.coffee_weight,
                    scale_offset=None,
                    scale_inv_variance=None,
                    scale_factor=None,
                )

        # Flush if batch is full
        if len(self.batch) >= self.batch_size:
            self.flush()

    def _add_record(
        self,
        timestamp_ms: int,
        received_at: float,
        message_type: str,
        state: str,
        weight: Optional[float],
        coffee_weight: Optional[float],
        scale_offset: Optional[float],
        scale_inv_variance: Optional[float],
        scale_factor: Optional[float],
    ) -> None:
        """Add a single record to the batch."""
        self.batch.append({
            "timestamp_ms": timestamp_ms,
            "received_at": received_at,
            "message_type": message_type,
            "state": state,
            "weight": weight,
            "coffee_weight": coffee_weight,
            "scale_offset": scale_offset,
            "scale_inv_variance": scale_inv_variance,
            "scale_factor": scale_factor,
        })

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
                existing_table = pq.read_table(self.output_path)
                table = pa.concat_tables([existing_table, table])
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

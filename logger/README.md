# Grindy Logger

WebSocket data logger for the Grindy coffee grinder controller. Connects to the microcontroller's WebSocket endpoint, deserializes binary messages, and logs time-series data to Apache Arrow/Parquet format.

## Features

- **Real-time logging** of weight readings and state changes
- **Binary deserialization** using Rust's postcard format
- **Arrow/Parquet output** for efficient time-series analysis
- **Ordered timestamp access** for data analysis
- **Auto-reconnection** on connection loss
- **Graceful shutdown** with Ctrl+C

## Installation

This project uses `uv` for dependency management. If you don't have `uv` installed:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then install the package:

```bash
cd logger
uv pip install -e .
```

## Usage

### Basic Usage

Connect to the default Grindy WebSocket and log data:

```bash
grindy-logger data.parquet
```

### Custom WebSocket URL

```bash
grindy-logger data.parquet --url ws://192.168.22.1/ws
```

### Adjust Batch Size

Write to disk every 200 records instead of default 100:

```bash
grindy-logger data.parquet --batch-size 200
```

### Stop Logging

Press `Ctrl+C` to gracefully shutdown. The logger will flush all buffered data before exiting.

## Output Format

Data is saved in Parquet format with the following schema:

| Column | Type | Description |
|--------|------|-------------|
| `timestamp_ms` | uint64 | Microcontroller timestamp in milliseconds |
| `received_at` | float64 | Local Unix timestamp when message was received |
| `message_type` | string | "Connected", "StateChange", or "WeightReading" |
| `state` | string | Grinder state: "Idle", "Stabilizing", "Grinding", "WaitingForRemoval" |
| `weight` | float32 | Total weight on scale (nullable) |
| `coffee_weight` | float32 | Coffee weight only (nullable) |
| `scale_offset` | float32 | Scale calibration offset (nullable) |
| `scale_inv_variance` | float32 | Inverse variance of scale (nullable) |
| `scale_factor` | float32 | Scale calibration factor (nullable) |

### Timestamp Ordering

All records are sorted by `timestamp_ms` to ensure correct temporal ordering for analysis. The `received_at` field allows you to detect timing discrepancies between device and host.

## Data Analysis

Load the Parquet file with pandas or polars:

```python
import pandas as pd

# Load data
df = pd.read_parquet("data.parquet")

# Filter to weight readings during grinding
grinding = df[(df["state"] == "Grinding") & (df["message_type"] == "WeightReading")]

# Plot coffee weight over time
import matplotlib.pyplot as plt
plt.plot(grinding["timestamp_ms"], grinding["coffee_weight"])
plt.xlabel("Time (ms)")
plt.ylabel("Coffee Weight (g)")
plt.show()
```

Or with polars for better performance:

```python
import polars as pl

# Load and filter
df = pl.read_parquet("data.parquet")
grinding = df.filter(
    (pl.col("state") == "Grinding") &
    (pl.col("message_type") == "WeightReading")
)

# Analyze grinding rate
grinding.select([
    "timestamp_ms",
    "coffee_weight",
    (pl.col("coffee_weight").diff() / pl.col("timestamp_ms").diff() * 1000).alias("grams_per_sec")
])
```

## Message Types

The WebSocket sends three types of messages:

1. **Connected**: Sent immediately on connection with current state and scale calibration
2. **StateChange**: Sent when grinder state transitions (Idle → Stabilizing → Grinding → WaitingForRemoval)
3. **WeightBatch**: Sent every ~100ms with up to 4 weight readings

## Troubleshooting

### Connection Refused

Ensure your computer is connected to the Grindy WiFi network:
- SSID: `grindy`
- Password: `grindyrockz`

### Empty Parquet File

The logger buffers data in memory before writing. If you stop the logger before the batch size is reached, use `Ctrl+C` to trigger a graceful shutdown that flushes buffered data.

## Development

Run in development mode:

```bash
cd logger
uv pip install -e .
python -m grindy_logger data.parquet
```

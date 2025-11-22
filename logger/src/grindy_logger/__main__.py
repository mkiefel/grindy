"""CLI entry point for Grindy data logger."""

import argparse
import asyncio
import sys
from pathlib import Path

from .client import run_client
from .writer import ArrowWriter


def main() -> None:
    """Main entry point for the CLI."""
    parser = argparse.ArgumentParser(
        description="Log data from Grindy coffee grinder to Arrow/Parquet file"
    )
    parser.add_argument(
        "output",
        type=str,
        help="Output Parquet file path (e.g., grindy_data.parquet)",
    )
    parser.add_argument(
        "--url",
        type=str,
        default="ws://192.168.1.9/ws",
        help="WebSocket URL (default: ws://192.168.1.9/ws)",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=100,
        help="Number of records to buffer before writing (default: 100)",
    )

    args = parser.parse_args()

    # Validate output path
    output_path = Path(args.output)
    if output_path.exists() and not output_path.is_file():
        print(f"Error: {args.output} exists but is not a file", file=sys.stderr)
        sys.exit(1)

    print(f"Grindy Data Logger")
    print(f"==================")
    print(f"WebSocket URL: {args.url}")
    print(f"Output file:   {args.output}")
    print(f"Batch size:    {args.batch_size}")
    print()

    # Create writer
    writer = ArrowWriter(args.output, batch_size=args.batch_size)

    # Setup message handler
    message_count = [0]  # Mutable container for closure

    def handle_message(message):
        """Handle incoming WebSocket message."""
        message_count[0] += 1
        writer.add_message(message)
        if message_count[0] % 10 == 0:
            print(f"Processed {message_count[0]} messages...", end="\r")

    # Run async client
    try:
        asyncio.run(run_client(args.url, handle_message))
    except KeyboardInterrupt:
        print("\nReceived interrupt signal")
    finally:
        print(f"\nFinalizing... Total messages processed: {message_count[0]}")
        writer.close()
        print("Done!")


if __name__ == "__main__":
    main()

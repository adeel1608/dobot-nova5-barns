#!/usr/bin/env python3
"""
POS_Integration.py
~~~~~~~~~~~~~~~~~~
Minimal Flask API that delegates POS parsing to pos_core.
"""

import json
import os
from pathlib import Path
from flask import Flask, request, jsonify
from dataclasses import asdict

from pos_core import parse_transaction, load_reference_data_from_db


app = Flask(__name__)


@app.route('/process-order', methods=['POST'])
def process_order():
    """Process POS order and return parsed transaction as JSON."""
    try:
        if not request.is_json:
            return jsonify({"error": "Content-Type must be application/json"}), 400

        order_data = request.get_json()

        # Validate required fields
        required_fields = ["transaction_id", "date", "time", "store_number", "pos_reg_id", "items"]
        for field in required_fields:
            if field not in order_data:
                return jsonify({"error": f"Missing required field: {field}"}), 400

        if not isinstance(order_data["items"], list) or len(order_data["items"]) == 0:
            return jsonify({"error": "Items must be a non-empty list"}), 400

        # Process the order via core
        parsed_order = parse_transaction(order_data)

        # Optional: persist the parsed dataclass to JSON for inspection
        try:
            tmp_dir = Path(__file__).parent / "temp_outputs"
            tmp_dir.mkdir(parents=True, exist_ok=True)
            out_path = tmp_dir / f"parsed_{parsed_order.transaction_id}.json"
            with open(out_path, "w", encoding="utf-8") as fp:
                json.dump(asdict(parsed_order), fp, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"WARN: failed to write temp parsed order JSON: {e}")

        return jsonify({"success": True, "parsed_order": asdict(parsed_order)})

    except Exception as e:
        return jsonify({"success": False, "error": f"Error processing order: {str(e)}"}), 500


def initialize_app():
    """Initialize the application by loading reference data into core."""
    success = load_reference_data_from_db(os.environ.get("POS_DB_PATH", "pos_reference.db"))
    if not success:
        print("Warning: Could not load reference data. API will run with empty data.")
    return success


if __name__ == "__main__":
    initialize_app()
    port = int(os.environ.get("PORT", 5000))
    debug = os.environ.get("DEBUG", "False").lower() == "true"
    print(f"Starting Dynamic POS Integration API on port {port}")
    app.run(host="0.0.0.0", port=port, debug=debug)
"""Read loop start positions from our existing People command file."""
from pathlib import Path


def loop_origins(filename, expected=None):
    result = {}
    for line in Path(filename).read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 5 and fields[1] == "GoTo":
            result[fields[0]] = tuple(float(value) for value in fields[2:5])
    if expected is None:
        expected = {"Upper_West", "Upper_East"} | {
            f"Lower_Cross_{i}" for i in range(1, 5)
        }
    if set(result) != expected:
        raise ValueError(f"Expected loop waypoints for {sorted(expected)}")
    return result

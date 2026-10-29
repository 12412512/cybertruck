"""Competition task-code parser. No robot motion or actuator commands."""

import re

COLORS = {1: "红色", 2: "黄色", 3: "蓝色", 4: "绿色", 5: "黑色", 6: "浅蓝"}
TASK_PATTERN = re.compile(r"([1-6]{3})\+([1-3]{3})\+([1-6]{3})\+([1-3]{3})", re.ASCII)


def parse_task(raw):
    """Parse page 6, preserving batch ordering and placement-group meanings."""
    if not isinstance(raw, str) or len(raw) > 64:
        raise ValueError("Task code must be a short text string")
    code = raw.strip()
    match = TASK_PATTERN.fullmatch(code)
    if match is None:
        raise ValueError("Expected CCC+PPP+CCC+PPP; colors 1-6, positions 1-3")
    first_colors, first_positions, second_colors, second_positions = match.groups()
    if len(set(first_colors)) != 3 or len(set(second_colors)) != 3:
        raise ValueError("Each batch must contain three different colors")
    if set(first_colors) != set(second_colors):
        raise ValueError("Batch 2 must match the colors of the first storage layer")
    if set(first_positions) != set("123") or set(second_positions) != set("123"):
        raise ValueError("Placement groups must be permutations of 123")
    storage_by_color = dict(zip(first_colors, first_positions))

    def batch(colors, positions, storage):
        items = []
        for index, (color, position) in enumerate(zip(colors, positions), start=1):
            item = {"order": index, "color_id": int(color),
                    "color_name": COLORS[int(color)], "rough_processing_ring": int(position)}
            item["temporary_storage_ring"] = int(position if storage else storage_by_color[color])
            items.append(item)
        return items

    # Group 4 applies only to rough processing. Storage layer 2 follows color.
    return {"valid": True, "raw": code,
            "batch1": batch(first_colors, first_positions, True),
            "batch2": batch(second_colors, second_positions, False)}


class TaskLatch:
    """Accept the same valid code in three successive decoded frames, once."""

    def __init__(self, required=3, max_gap=1.5):
        self.required = required
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.accepted = None
        self.pending = None
        self.count = 0
        self.last_seen = 0.0

    def observe(self, codes, now):
        codes = list(dict.fromkeys(code for code in codes if code))
        if self.accepted is not None:
            return "ACCEPTED", None
        if len(codes) != 1:
            self.pending, self.count = None, 0
            return ("AMBIGUOUS" if codes else "SEARCHING"), None
        try:
            task = parse_task(codes[0])
        except ValueError:
            self.pending, self.count = None, 0
            return "INVALID_CODE", None
        if task["raw"] != self.pending or now - self.last_seen > self.max_gap:
            self.pending, self.count = task["raw"], 0
        self.count += 1
        self.last_seen = now
        if self.count >= self.required:
            self.accepted = task
            return "ACCEPTED", task
        return "CONFIRMING", None

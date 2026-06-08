import math
from pathlib import Path
import xml.etree.ElementTree as ET


MAP_PATH = (
    Path(__file__).resolve().parents[1]
    / "maps"
    / "carmaker_builtin_urban"
    / "lanelet2_map.osm"
)


def _load_centerline(osm_path: Path):
    root = ET.parse(osm_path).getroot()

    nodes = {}
    for node in root.findall("node"):
        tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            nodes[node.attrib["id"]] = (float(tags["local_x"]), float(tags["local_y"]))

    ways = {
        way.attrib["id"]: [nd.attrib["ref"] for nd in way.findall("nd")]
        for way in root.findall("way")
    }

    for relation in root.findall("relation"):
        tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        for member in relation.findall("member"):
            if member.attrib.get("role") != "centerline":
                continue
            return [nodes[node_id] for node_id in ways[member.attrib["ref"]]]

    raise AssertionError(f"no centerline relation found in {osm_path}")


def _segment_length(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _angle_delta(a, b):
    delta = a - b
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return abs(delta)


def test_stage_b_centerline_has_no_sparse_route_jumps_before_stage_b_stop_margin():
    centerline = _load_centerline(MAP_PATH)

    cumulative = 0.0
    previous_heading = None
    failures = []
    for index in range(1, len(centerline)):
        segment_start = cumulative
        segment = _segment_length(centerline[index - 1], centerline[index])
        cumulative += segment
        if segment_start > 900.0:
            break

        if segment > 5.0:
            failures.append(
                f"segment {index - 1}->{index} is {segment:.3f}m at s={segment_start:.3f}"
            )

        if index < len(centerline) - 1:
            heading = _heading(centerline[index], centerline[index + 1])
            if previous_heading is not None:
                delta_deg = math.degrees(_angle_delta(heading, previous_heading))
                if delta_deg > 35.0:
                    failures.append(
                        f"heading jump at index {index}: {delta_deg:.2f}deg at s={cumulative:.3f}"
                    )
            previous_heading = heading

    assert not failures

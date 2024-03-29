use std::collections::HashSet;

use anyhow::Result;
use geojson::Feature;
use geom::{Polygon, Ring};

use crate::{IntersectionID, LaneType, RoadID, StreetNetwork};

/// A "tight" cycle of roads and intersections, with a polygon capturing the negative space inside.
pub struct Block {
    pub kind: BlockKind,
    pub steps: Vec<Step>,
    pub polygon: Polygon,
}

#[derive(Clone, Copy)]
pub enum Step {
    Node(IntersectionID),
    Edge(RoadID),
}

#[derive(Debug)]
pub enum BlockKind {
    /// The space between a road and sidewalk. It might be a wide sidewalk or contain grass.
    RoadAndSidewalk,
    /// The space between a road and cycle lane. It should contain some kind of separation.
    RoadAndCycleLane,
    /// The space between a cycle lane and sidewalk. It should contain some kind of separation --
    /// at least a curb.
    CycleLaneAndSidewalk,
    /// The space between one-way roads. Probably has some kind of physical barrier, not just
    /// markings.
    DualCarriageway,
    Unknown,
}

impl StreetNetwork {
    // Start at road's src_i
    pub fn find_block(&self, start: RoadID, left: bool) -> Result<Block> {
        // TODO ??
        let clockwise = left;
        let steps = walk_around(self, start, clockwise)?;
        let polygon = trace_polygon(self, &steps, clockwise)?;
        let kind = classify(self, &steps);

        Ok(Block {
            kind,
            steps,
            polygon,
        })
    }

    pub fn find_all_blocks(&self) -> Result<String> {
        // TODO consider a Left/Right enum instead of bool
        let mut visited_roads: HashSet<(RoadID, bool)> = HashSet::new();
        let mut blocks = Vec::new();

        for r in self.roads.keys() {
            for left in [true, false] {
                if visited_roads.contains(&(*r, left)) {
                    continue;
                }
                if let Ok(block) = self.find_block(*r, left) {
                    // TODO Put more info in Step to avoid duplicating logic with trace_polygon
                    for pair in block.steps.windows(2) {
                        match (pair[0], pair[1]) {
                            (Step::Edge(r), Step::Node(i)) => {
                                let road = &self.roads[&r];
                                if road.dst_i == i {
                                    visited_roads.insert((r, left));
                                } else {
                                    visited_roads.insert((r, !left));
                                }
                            }
                            // Skip... unless for the last case?
                            (Step::Node(_), Step::Edge(_)) => {}
                            _ => unreachable!(),
                        }
                    }
                    blocks.push(block);
                }
            }
        }

        let mut features = Vec::new();
        for block in blocks {
            let mut f = Feature::from(block.polygon.to_geojson(Some(&self.gps_bounds)));
            f.set_property("type", "block");
            f.set_property("kind", format!("{:?}", block.kind));
            features.push(f);
        }
        serialize_features(features)
    }
}

impl Block {
    pub fn render_polygon(&self, streets: &StreetNetwork) -> Result<String> {
        let mut f = Feature::from(self.polygon.to_geojson(Some(&streets.gps_bounds)));
        f.set_property("type", "block");
        f.set_property("kind", format!("{:?}", self.kind));
        serialize_features(vec![f])
    }

    pub fn render_debug(&self, streets: &StreetNetwork) -> Result<String> {
        let mut features = Vec::new();

        for step in &self.steps {
            match step {
                Step::Node(i) => {
                    let mut f = Feature::from(
                        streets.intersections[&i]
                            .polygon
                            .to_geojson(Some(&streets.gps_bounds)),
                    );
                    f.set_property("type", "intersection");
                    features.push(f);
                }
                Step::Edge(r) => {
                    let road = &streets.roads[&r];
                    let mut f = Feature::from(
                        road.center_line
                            .make_polygons(road.total_width())
                            .to_geojson(Some(&streets.gps_bounds)),
                    );
                    f.set_property("type", "road");
                    features.push(f);
                }
            }
        }

        serialize_features(features)
    }
}

fn walk_around(streets: &StreetNetwork, start_road: RoadID, clockwise: bool) -> Result<Vec<Step>> {
    let start_i = streets.roads[&start_road].src_i;

    let mut current_i = start_i;
    let mut current_r = start_road;

    let mut steps = vec![Step::Edge(current_r)];

    while current_i != start_i || steps.len() < 2 {
        // Fail for dead-ends (for now, to avoid tracing around the entire clipped map)
        if streets.intersections[&current_i].roads.len() == 1 {
            bail!("Found a dead-end at {current_i}");
        }

        let next_i = &streets.intersections[&streets.roads[&current_r].other_side(current_i)];
        let idx = next_i.roads.iter().position(|x| *x == current_r).unwrap();
        let next_idx = if clockwise {
            if idx == next_i.roads.len() - 1 {
                0
            } else {
                idx + 1
            }
        } else {
            if idx == 0 {
                next_i.roads.len() - 1
            } else {
                idx - 1
            }
        };
        let next_r = next_i.roads[next_idx];
        steps.push(Step::Node(next_i.id));
        steps.push(Step::Edge(next_r));
        current_i = next_i.id;
        current_r = next_r;
    }

    Ok(steps)
}

fn trace_polygon(streets: &StreetNetwork, steps: &Vec<Step>, clockwise: bool) -> Result<Polygon> {
    let shift_dir = if clockwise { -1.0 } else { 1.0 };
    let mut pts = Vec::new();

    // steps will begin and end with an edge
    for pair in steps.windows(2) {
        match (pair[0], pair[1]) {
            (Step::Edge(r), Step::Node(i)) => {
                let road = &streets.roads[&r];
                if road.dst_i == i {
                    pts.extend(
                        road.center_line
                            .shift_either_direction(shift_dir * road.half_width())?
                            .into_points(),
                    );
                } else {
                    pts.extend(
                        road.center_line
                            .reversed()
                            .shift_either_direction(shift_dir * road.half_width())?
                            .into_points(),
                    );
                }
            }
            // Skip... unless for the last case?
            (Step::Node(_), Step::Edge(_)) => {}
            _ => unreachable!(),
        }
    }

    pts.push(pts[0]);
    Ok(Ring::deduping_new(pts)?.into_polygon())
}

fn classify(streets: &StreetNetwork, steps: &Vec<Step>) -> BlockKind {
    let mut has_road = false;
    let mut has_cycle_lane = false;
    let mut has_sidewalk = false;

    for step in steps {
        if let Step::Edge(r) = step {
            let road = &streets.roads[r];
            if road.is_driveable() {
                // TODO Or bus lanes?
                has_road = true;
            } else if road.lane_specs_ltr.len() == 1
                && road.lane_specs_ltr[0].lt == LaneType::Biking
            {
                has_cycle_lane = true;
            } else if road.lane_specs_ltr.len() == 1
                && road.lane_specs_ltr[0].lt == LaneType::Sidewalk
            {
                has_sidewalk = true;
            }
        }
    }

    if has_road && has_sidewalk {
        return BlockKind::RoadAndSidewalk;
    }
    if has_road && has_cycle_lane {
        return BlockKind::RoadAndCycleLane;
    }
    if has_road {
        // TODO Insist on one-ways pointing the opposite direction? What about different types of
        // small connector roads?
        return BlockKind::DualCarriageway;
    }
    // TODO This one is usually missed, because of a small piece of road crossing both
    if !has_road && has_cycle_lane && has_sidewalk {
        return BlockKind::CycleLaneAndSidewalk;
    }

    BlockKind::Unknown
}

fn serialize_features(features: Vec<Feature>) -> Result<String> {
    let gj = geojson::GeoJson::from(geojson::FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    });
    let output = serde_json::to_string_pretty(&gj)?;
    Ok(output)
}

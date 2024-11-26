use anyhow::Result;
use geojson::Feature;
use geom::{ArrowCap, Distance, Line, PolyLine, Polygon};
use osm2lanes::TurnDirection;

use super::{serialize_features, Filter};
use crate::{
    BufferType, Direction, DrivingSide, LaneSpec, LaneType, ParkingType, Road, StreetNetwork,
};

impl StreetNetwork {
    /// Generate polygons representing lane markings, with a property indicating type.
    pub fn to_lane_markings_geojson(&self, filter: &Filter) -> Result<String> {
        // TODO Split this up!
        let gps_bounds = Some(&self.gps_bounds);
        let mut features = Vec::new();

        for road in filter.roads(self) {
            // Always oriented in the direction of the road
            let mut lane_centers = road.get_lane_center_lines();

            for (idx, pair) in road.lane_specs_ltr.windows(2).enumerate() {
                // Generate a "center line" between lanes of different directions
                if pair[0].dir != pair[1].dir {
                    let between = lane_centers[idx].shift_right(pair[0].width / 2.0)?;
                    // TODO Ideally we would return a full LineString, and the caller would choose
                    // how to style these as thickened dashed lines.
                    // TODO We could also at least return a MultiPolygon here
                    for poly in between.dashed_lines(
                        Distance::meters(0.25),
                        Distance::meters(2.0),
                        Distance::meters(1.0),
                    ) {
                        let mut f = Feature::from(poly.to_geojson(gps_bounds));
                        f.set_property("type", "center line");
                        f.set_property("layer", road.layer);
                        features.push(f);
                    }
                    continue;
                }

                // Generate a "lane separator" between driving lanes only
                if pair[0].lt == LaneType::Driving && pair[1].lt == LaneType::Driving {
                    let between = lane_centers[idx].shift_right(pair[0].width / 2.0)?;
                    for poly in between.dashed_lines(
                        Distance::meters(0.25),
                        Distance::meters(1.0),
                        Distance::meters(1.5),
                    ) {
                        let mut f = Feature::from(poly.to_geojson(gps_bounds));
                        f.set_property("type", "lane separator");
                        f.set_property("layer", road.layer);
                        features.push(f);
                    }
                }
            }

            // Stop line distances are relative to the direction of the road, not the lane!
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                for (polygon, kind) in draw_stop_lines(lane, center, road) {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", kind);
                    features.push(f);
                }
            }

            // Below renderings need lane centers to point in the direction of the lane
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter_mut()) {
                if lane.dir == Direction::Backward {
                    *center = center.reversed();
                }
            }

            // Draw arrows along any travel lane
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if !lane.lt.is_for_moving_vehicles() {
                    continue;
                }

                let step_size = Distance::meters(20.0);
                let buffer_ends = Distance::meters(5.0);
                let arrow_len = Distance::meters(3.5);
                let thickness = Distance::meters(0.25);
                let turn_arrow_len = Distance::meters(2.5);

                for (pt, angle) in center.step_along(step_size, buffer_ends) {
                    let allowed_turns: Vec<_> = lane.allowed_turns.iter().collect();
                    match allowed_turns.as_slice() {
                        [] | [TurnDirection::Through] => {
                            // 直箭头
                            let arrow = PolyLine::must_new(vec![
                                pt.project_away(arrow_len / 2.0, angle.opposite()),
                                pt.project_away(arrow_len / 2.0, angle),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f = Feature::from(arrow.to_geojson(gps_bounds));
                            f.set_property("type", "lane arrow");
                            f.set_property("layer", road.layer);
                            features.push(f);
                        }
                        [TurnDirection::Left] => {
                            // draw arrow
                            let offset_pt = pt
                                .project_away(Distance::meters(2.1), angle) // move up
                                .project_away(Distance::meters(0.5), angle.rotate_degs(90.0)); // move right
                            let arrow = PolyLine::must_new(vec![
                                offset_pt.project_away(turn_arrow_len / 2.0, angle.opposite()),
                                offset_pt
                                    .project_away(turn_arrow_len / 2.0, angle.rotate_degs(-110.0)),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_arrow = Feature::from(arrow.to_geojson(gps_bounds));
                            f_arrow.set_property("type", "lane arrow");
                            f_arrow.set_property("layer", road.layer);
                            features.push(f_arrow);

                            // draw rectangle
                            let rect_width = Distance::meters(0.5);
                            let rect_len = Distance::meters(1.4);

                            let offset_pt = pt
                                .project_away(Distance::meters(0.35), angle.opposite()) // move down
                                .project_away(Distance::meters(0.4), angle.rotate_degs(90.0)); // move right
                            let rec = PolyLine::must_new(vec![
                                offset_pt.project_away(rect_len, angle.opposite()),
                                offset_pt.project_away(rect_len, angle),
                            ])
                            .make_polygons(rect_width);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_rect = Feature::from(rec.to_geojson(gps_bounds));
                            f_rect.set_property("type", "lane arrow");
                            f_rect.set_property("layer", road.layer);
                            features.push(f_rect);
                        }
                        [TurnDirection::Right] => {
                            // draw arrow
                            let offset_pt = pt
                                .project_away(Distance::meters(2.1), angle) // move up
                                .project_away(Distance::meters(0.5), angle.rotate_degs(-90.0)); // move right
                            let arrow = PolyLine::must_new(vec![
                                offset_pt.project_away(turn_arrow_len / 2.0, angle.opposite()),
                                offset_pt
                                    .project_away(turn_arrow_len / 2.0, angle.rotate_degs(110.0)),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_arrow = Feature::from(arrow.to_geojson(gps_bounds));
                            f_arrow.set_property("type", "lane arrow");
                            f_arrow.set_property("layer", road.layer);
                            features.push(f_arrow);

                            // draw rectangle
                            let rect_width = Distance::meters(0.5);
                            let rect_len = Distance::meters(1.4);

                            let offset_pt = pt
                                .project_away(Distance::meters(0.35), angle.opposite()) // move down
                                .project_away(Distance::meters(0.4), angle.rotate_degs(-90.0)); // move right
                            let rec = PolyLine::must_new(vec![
                                offset_pt.project_away(rect_len, angle.opposite()),
                                offset_pt.project_away(rect_len, angle),
                            ])
                            .make_polygons(rect_width);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_rect = Feature::from(rec.to_geojson(gps_bounds));
                            f_rect.set_property("type", "lane arrow");
                            f_rect.set_property("layer", road.layer);
                            features.push(f_rect);
                        }
                        [TurnDirection::Through, TurnDirection::Left]
                        | [TurnDirection::Left, TurnDirection::Through] => {
                            let turn_arrow_len = Distance::meters(3.0);

                            // arrow straight
                            let offset_pt =
                                pt.project_away(Distance::meters(0.3), angle.rotate_degs(90.0)); // 偏移10个单位
                            let arrow_t = PolyLine::must_new(vec![
                                offset_pt.project_away(arrow_len / 2.0, angle.opposite()),
                                offset_pt.project_away(arrow_len / 2.0, angle),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_t = Feature::from(arrow_t.to_geojson(gps_bounds));
                            f_t.set_property("type", "lane arrow");
                            f_t.set_property("layer", road.layer);
                            features.push(f_t);

                            // arrow right
                            let offset_pt = pt
                                .project_away(Distance::meters(0.2), angle.rotate_degs(90.0))
                                .project_away(Distance::meters(0.8), angle); // 偏移10个单位
                            let arrow_r = PolyLine::must_new(vec![
                                offset_pt.project_away(turn_arrow_len / 2.0, angle.opposite()),
                                offset_pt
                                    .project_away(turn_arrow_len / 2.0, angle.rotate_degs(-110.0)),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_l = Feature::from(arrow_r.to_geojson(gps_bounds));
                            f_l.set_property("type", "lane arrow");
                            f_l.set_property("layer", road.layer);
                            features.push(f_l);
                        }
                        [TurnDirection::Through, TurnDirection::Right]
                        | [TurnDirection::Right, TurnDirection::Through] => {
                            let turn_arrow_len = Distance::meters(3.0);

                            // arrow straight
                            let offset_pt =
                                pt.project_away(Distance::meters(0.3), angle.rotate_degs(-90.0)); // 偏移10个单位
                            let arrow_t = PolyLine::must_new(vec![
                                offset_pt.project_away(arrow_len / 2.0, angle.opposite()),
                                offset_pt.project_away(arrow_len / 2.0, angle),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_t = Feature::from(arrow_t.to_geojson(gps_bounds));
                            f_t.set_property("type", "lane arrow");
                            f_t.set_property("layer", road.layer);
                            features.push(f_t);

                            // arrow right
                            let offset_pt = pt
                                .project_away(Distance::meters(0.2), angle.rotate_degs(-90.0))
                                .project_away(Distance::meters(0.8), angle); // 偏移10个单位
                            let arrow_r = PolyLine::must_new(vec![
                                offset_pt.project_away(turn_arrow_len / 2.0, angle.opposite()),
                                offset_pt
                                    .project_away(turn_arrow_len / 2.0, angle.rotate_degs(110.0)),
                            ])
                            .make_arrow(thickness * 2.0, ArrowCap::Triangle);
                            // .get_outer_ring()
                            // .to_outline(thickness / 2.0);
                            let mut f_r = Feature::from(arrow_r.to_geojson(gps_bounds));
                            f_r.set_property("type", "lane arrow");
                            f_r.set_property("layer", road.layer);
                            features.push(f_r);
                        }
                        _ => continue, // 其他转向类型暂不处理
                    }
                }
            }

            // Add stripes to show most buffers.
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if !matches!(lane.lt, LaneType::Buffer(_)) {
                    continue;
                }
                if lane.lt == LaneType::Buffer(BufferType::Curb) {
                    continue;
                }

                // Mark the sides of the lane clearly
                let thickness = Distance::meters(0.25);
                for dir in [-1.0, 1.0] {
                    let mut f = Feature::from(
                        center
                            .shift_either_direction(dir * (lane.width - thickness) / 2.0)
                            .unwrap()
                            .make_polygons(thickness)
                            .to_geojson(gps_bounds),
                    );
                    f.set_property("type", "buffer edge");
                    f.set_property("layer", road.layer);
                    features.push(f);
                }

                // Diagonal stripes along the lane
                let step_size = Distance::meters(3.0);
                let buffer_ends = Distance::meters(5.0);
                for (center, angle) in center.step_along(step_size, buffer_ends) {
                    // Extend the stripes into the side lines
                    let left =
                        center.project_away(lane.width / 2.0 + thickness, angle.rotate_degs(45.0));
                    let right = center.project_away(
                        lane.width / 2.0 + thickness,
                        angle.rotate_degs(45.0).opposite(),
                    );
                    let mut f = Feature::from(
                        Line::must_new(left, right)
                            .make_polygons(thickness)
                            .to_geojson(gps_bounds),
                    );
                    f.set_property("type", "buffer stripe");
                    f.set_property("layer", road.layer);
                    features.push(f);
                }
            }

            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                let polygons = match lane.lt {
                    LaneType::Parking(ParkingType::Parallel) => {
                        draw_parallel_parking_lines(lane, center, self)
                    }
                    LaneType::Parking(ParkingType::Diagonal) => {
                        draw_diagonal_parking_lines(lane, center, self)
                    }
                    LaneType::Parking(ParkingType::Perpendicular) => {
                        draw_perpendicular_parking_lines(lane, center, self)
                    }
                    _ => continue,
                };
                for polygon in polygons {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", "parking hatch");
                    features.push(f);
                }
            }

            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if lane.lt != LaneType::Sidewalk {
                    continue;
                }
                for polygon in draw_sidewalk_lines(lane, center) {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", "sidewalk line");
                    features.push(f);
                }
            }

            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if lane.lt != LaneType::SharedUse && lane.lt != LaneType::Footway {
                    continue;
                }
                for polygon in draw_path_outlines(lane, center) {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", "path outline");
                    features.push(f);
                }
            }
        }

        serialize_features(features)
    }
}

fn draw_stop_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    road: &Road,
) -> Vec<(Polygon, &'static str)> {
    let mut results = Vec::new();

    if !matches!(
        lane.lt,
        LaneType::Driving | LaneType::Bus | LaneType::Biking
    ) {
        return results;
    }
    let thickness = Distance::meters(0.5);

    let stop_line = if lane.dir == Direction::Forward {
        &road.stop_line_end
    } else {
        &road.stop_line_start
    };

    // The vehicle line
    if let Some(dist) = stop_line.vehicle_distance {
        if let Ok((pt, angle)) = center.dist_along(dist) {
            results.push((
                Line::must_new(
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(90.0)),
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(-90.0)),
                )
                .make_polygons(thickness),
                "vehicle stop line",
            ));
        }
    }

    if let Some(dist) = stop_line.bike_distance {
        if let Ok((pt, angle)) = center.dist_along(dist) {
            results.push((
                Line::must_new(
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(90.0)),
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(-90.0)),
                )
                .make_polygons(thickness),
                "bike stop line",
            ));
        }
    }

    // TODO Change the rendering based on interruption too

    results
}

fn draw_parallel_parking_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    streets: &StreetNetwork,
) -> Vec<Polygon> {
    let mut result = Vec::new();

    // No spots next to intersections
    let spots =
        (center.length() / streets.config.parallel_street_parking_spot_length).floor() - 2.0;
    let num_spots = if spots >= 1.0 {
        spots as usize
    } else {
        return result;
    };

    let leg_length = Distance::meters(1.0);
    for idx in 0..=num_spots {
        let (pt, lane_angle) = center.must_dist_along(
            streets.config.parallel_street_parking_spot_length * (1.0 + idx as f64),
        );
        let perp_angle = if streets.config.driving_side == DrivingSide::Right {
            lane_angle.rotate_degs(270.0)
        } else {
            lane_angle.rotate_degs(90.0)
        };
        // Find the outside of the lane. Actually, shift inside a little bit, since the line
        // will have thickness, but shouldn't really intersect the adjacent line
        // when drawn.
        let t_pt = pt.project_away(lane.width * 0.4, perp_angle);
        // The perp leg
        let p1 = t_pt.project_away(leg_length, perp_angle.opposite());
        result.push(Line::must_new(t_pt, p1).make_polygons(Distance::meters(0.25)));
        // Upper leg
        let p2 = t_pt.project_away(leg_length, lane_angle);
        result.push(Line::must_new(t_pt, p2).make_polygons(Distance::meters(0.25)));
        // Lower leg
        let p3 = t_pt.project_away(leg_length, lane_angle.opposite());
        result.push(Line::must_new(t_pt, p3).make_polygons(Distance::meters(0.25)));
    }

    result
}

fn draw_diagonal_parking_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    streets: &StreetNetwork,
) -> Vec<Polygon> {
    let mut result = Vec::new();

    // No spots next to intersections
    // TODO This needs to account for the 45 degree angle too
    let spots = (center.length() / streets.config.vehicle_width_for_parking_spots).floor() - 2.0;
    let num_spots = if spots >= 1.0 {
        spots as usize
    } else {
        return result;
    };

    // TODO Would PolyLine::step_along be simpler?
    for idx in 0..=num_spots {
        let (pt, lane_angle) = center
            .must_dist_along(streets.config.vehicle_width_for_parking_spots * (1.0 + idx as f64));
        let offset = if lane.dir == Direction::Forward {
            -45.0
        } else {
            45.0
        };
        let diag_angle = if streets.config.driving_side == DrivingSide::Right {
            lane_angle.rotate_degs(270.0 + offset)
        } else {
            lane_angle.rotate_degs(90.0 + offset)
        };
        // Find the inside and outside of the lane
        // TODO Do trig, the length changes
        let outside_pt = pt.project_away(lane.width * 0.4, diag_angle);
        let inside_pt = pt.project_away(lane.width * 0.5, diag_angle.opposite());
        result.push(Line::must_new(outside_pt, inside_pt).make_polygons(Distance::meters(0.25)));
    }

    result
}

fn draw_perpendicular_parking_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    streets: &StreetNetwork,
) -> Vec<Polygon> {
    let mut result = Vec::new();

    // No spots next to intersections
    let spots = (center.length() / streets.config.vehicle_width_for_parking_spots).floor() - 2.0;
    let num_spots = if spots >= 1.0 {
        spots as usize
    } else {
        return result;
    };

    for idx in 0..=num_spots {
        let (pt, lane_angle) = center
            .must_dist_along(streets.config.vehicle_width_for_parking_spots * (1.0 + idx as f64));
        let perp_angle = if streets.config.driving_side == DrivingSide::Right {
            lane_angle.rotate_degs(270.0)
        } else {
            lane_angle.rotate_degs(90.0)
        };
        // Find the inside and outside of the lane
        let outside_pt = pt.project_away(lane.width * 0.4, perp_angle);
        let inside_pt = pt.project_away(lane.width * 0.5, perp_angle.opposite());
        result.push(Line::must_new(outside_pt, inside_pt).make_polygons(Distance::meters(0.25)));
    }

    result
}

fn draw_sidewalk_lines(lane: &LaneSpec, center: &PolyLine) -> Vec<Polygon> {
    center
        .step_along(lane.width, lane.width)
        .into_iter()
        .map(|(pt, angle)| {
            // Project away an arbitrary amount
            let pt2 = pt.project_away(Distance::meters(1.0), angle);
            perp_line(Line::must_new(pt, pt2), lane.width).make_polygons(Distance::meters(0.25))
        })
        .collect()
}

fn draw_path_outlines(lane: &LaneSpec, center: &PolyLine) -> Vec<Polygon> {
    let mut result = Vec::new();
    // Dashed lines on both sides
    for dir in [-1.0, 1.0] {
        let pl = center
            .shift_either_direction(dir * lane.width / 2.0)
            .unwrap();
        result.extend(pl.exact_dashed_polygons(
            Distance::meters(0.25),
            Distance::meters(1.0),
            Distance::meters(1.5),
        ));
    }
    result
}

// this always does it at pt1
fn perp_line(l: Line, length: Distance) -> Line {
    let pt1 = l.shift_right(length / 2.0).pt1();
    let pt2 = l.shift_left(length / 2.0).pt1();
    Line::must_new(pt1, pt2)
}

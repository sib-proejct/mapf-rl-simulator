//! Physical occupancy and deterministic, non-expiring passage rights.
//!
//! This allocator is independent of the collision kernel. Call it once for the
//! whole fleet, before advancing any robot. Only observed clearance releases a
//! resource; simulation time is used solely to order waiting requests.

use std::collections::{BTreeMap, BTreeSet};

pub type Cell = (u32, u32);
pub type Resources = BTreeSet<Cell>;

pub fn corridor_resources(map: &crate::world::GridMap, cell: Cell) -> Resources {
    fn neighbors(map: &crate::world::GridMap, cell: Cell) -> Vec<Cell> {
        [(1_i64, 0_i64), (-1, 0), (0, 1), (0, -1)]
            .into_iter()
            .filter_map(|(dx, dy)| {
                let x = u32::try_from(i64::from(cell.0) + dx).ok()?;
                let y = u32::try_from(i64::from(cell.1) + dy).ok()?;
                (map.is_blocked(crate::world::GridCell::new(x, y)) == Ok(false)).then_some((x, y))
            })
            .collect()
    }
    let mut resources = BTreeSet::from([cell]);
    if neighbors(map, cell).len() != 2 {
        return resources;
    }
    let mut pending = vec![cell];
    while let Some(current) = pending.pop() {
        for next in neighbors(map, current) {
            if resources.insert(next) && neighbors(map, next).len() == 2 {
                pending.push(next);
            }
        }
    }
    resources
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Request {
    pub robot_id: String,
    pub resources: Resources,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Allocation {
    pub granted: BTreeSet<String>,
    pub blockers: BTreeMap<String, BTreeSet<String>>,
    pub cycle: BTreeSet<String>,
}

impl Allocation {
    pub(crate) fn refresh_cycle(&mut self) {
        self.cycle.clear();
        // Reachability back to the origin identifies only cycle participants,
        // without treating ordinary queues or long waits as failures.
        for origin in self.blockers.keys() {
            let mut pending: Vec<_> = self.blockers[origin].iter().cloned().collect();
            let mut visited = BTreeSet::new();
            while let Some(id) = pending.pop() {
                if &id == origin {
                    self.cycle.insert(origin.clone());
                    break;
                }
                if visited.insert(id.clone())
                    && let Some(next) = self.blockers.get(&id)
                {
                    pending.extend(next.iter().cloned());
                }
            }
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct PassageRights {
    grants: BTreeMap<String, Resources>,
    waiting_since: BTreeMap<String, u64>,
    tick: u64,
}

impl PassageRights {
    /// Clear waiting age only after the fleet's forecast checks accept motion.
    pub(crate) fn confirm(&mut self, granted: &BTreeSet<String>) {
        self.waiting_since.retain(|id, _| !granted.contains(id));
    }

    /// Install every robot's observed footprint and braking envelope before
    /// allocating extensions. These resources never expire by time.
    pub fn allocate(
        &mut self,
        occupied: &BTreeMap<String, Resources>,
        retained: &BTreeMap<String, Resources>,
        requests: &[Request],
    ) -> Allocation {
        self.tick = self.tick.saturating_add(1);
        self.grants.clone_from(retained);
        let requested: BTreeSet<_> = requests.iter().map(|r| r.robot_id.clone()).collect();
        self.waiting_since.retain(|id, _| requested.contains(id));
        for request in requests {
            self.waiting_since
                .entry(request.robot_id.clone())
                .or_insert(self.tick);
        }
        let mut ordered: Vec<_> = requests.iter().collect();
        ordered.sort_by_key(|r| (self.waiting_since[&r.robot_id], &r.robot_id));
        let mut result = Allocation::default();
        for request in ordered {
            let mut blockers = BTreeSet::new();
            for (id, resources) in occupied.iter().chain(self.grants.iter()) {
                if id != &request.robot_id && !request.resources.is_disjoint(resources) {
                    blockers.insert(id.clone());
                }
            }
            if blockers.is_empty() {
                self.grants
                    .entry(request.robot_id.clone())
                    .or_default()
                    .extend(request.resources.iter().copied());
                result.granted.insert(request.robot_id.clone());
            } else {
                result.blockers.insert(request.robot_id.clone(), blockers);
            }
        }
        result.refresh_cycle();
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn refreshed_forecast_cycle_excludes_queued_robots_and_clears_old_cycles() {
        let mut allocation = Allocation {
            blockers: BTreeMap::from([
                ("a".into(), BTreeSet::from(["b".into()])),
                ("b".into(), BTreeSet::from(["a".into()])),
                ("queued".into(), BTreeSet::from(["a".into()])),
            ]),
            ..Allocation::default()
        };
        allocation.refresh_cycle();
        assert_eq!(allocation.cycle, BTreeSet::from(["a".into(), "b".into()]));
        allocation.blockers.remove("b");
        allocation.refresh_cycle();
        assert!(allocation.cycle.is_empty());
    }
    fn cells(values: &[Cell]) -> Resources {
        values.iter().copied().collect()
    }
    fn request(id: &str, values: &[Cell]) -> Request {
        Request {
            robot_id: id.into(),
            resources: cells(values),
        }
    }
    #[test]
    fn atomic_destination_and_opposite_edge_conflicts() {
        let mut rights = PassageRights::default();
        let occupied = BTreeMap::from([
            ("a".into(), cells(&[(0, 0)])),
            ("b".into(), cells(&[(2, 0)])),
        ]);
        let requests = [
            request("b", &[(2, 0), (1, 0)]),
            request("a", &[(0, 0), (1, 0)]),
        ];
        let first = rights.allocate(&occupied, &BTreeMap::new(), &requests);
        assert_eq!(first.granted, BTreeSet::from(["a".into()]));
        assert_eq!(first.blockers["b"], BTreeSet::from(["a".into()]));
        let swap = rights.allocate(
            &occupied,
            &BTreeMap::new(),
            &[
                request("a", &[(0, 0), (2, 0)]),
                request("b", &[(2, 0), (0, 0)]),
            ],
        );
        assert!(swap.granted.is_empty());
        assert_eq!(swap.cycle, BTreeSet::from(["a".into(), "b".into()]));
    }
    #[test]
    fn physical_clearance_and_fifo_have_no_deadline() {
        let mut rights = PassageRights::default();
        let occupied = BTreeMap::from([("parked".into(), cells(&[(1, 0)]))]);
        let old = request("z", &[(1, 0)]);
        for _ in 0..1000 {
            assert!(
                rights
                    .allocate(&occupied, &BTreeMap::new(), std::slice::from_ref(&old))
                    .granted
                    .is_empty()
            );
        }
        let resumed = rights.allocate(
            &BTreeMap::new(),
            &BTreeMap::new(),
            &[request("a", &[(1, 0)]), old],
        );
        assert_eq!(resumed.granted, BTreeSet::from(["z".into()]));
        let braking = rights.allocate(
            &BTreeMap::new(),
            &BTreeMap::from([("z".into(), cells(&[(1, 0)]))]),
            &[request("a", &[(1, 0)])],
        );
        assert!(braking.granted.is_empty());
        let stopped = rights.allocate(
            &BTreeMap::new(),
            &BTreeMap::new(),
            &[request("a", &[(1, 0)])],
        );
        assert!(stopped.granted.contains("a"));
    }
    #[test]
    fn moving_grants_extend_and_release_only_observed_clearance() {
        let mut rights = PassageRights::default();
        let first = rights.allocate(
            &BTreeMap::new(),
            &BTreeMap::new(),
            &[request("a", &[(0, 0), (1, 0)])],
        );
        assert!(first.granted.contains("a"));
        let retained = BTreeMap::from([("a".into(), cells(&[(1, 0)]))]);
        let next = rights.allocate(
            &retained,
            &retained,
            &[
                request("b", &[(0, 0)]),
                request("a", &[(1, 0), (2, 0)]),
                request("c", &[(1, 0)]),
            ],
        );
        assert!(next.granted.contains("a"));
        assert!(next.granted.contains("b"));
        assert_eq!(next.blockers["c"], BTreeSet::from(["a".into()]));
        assert_eq!(rights.grants["a"], cells(&[(1, 0), (2, 0)]));
    }
    #[test]
    fn denied_extension_keeps_braking_rights_and_waiting_age() {
        let mut rights = PassageRights::default();
        let retained = BTreeMap::from([("a".into(), cells(&[(1, 0), (2, 0)]))]);
        let occupied = BTreeMap::from([("parked".into(), cells(&[(3, 0)]))]);
        let blocked = rights.allocate(
            &occupied,
            &retained,
            &[request("a", &[(1, 0), (2, 0), (3, 0)])],
        );
        assert!(!blocked.granted.contains("a"));
        assert_eq!(rights.grants["a"], retained["a"]);
        let waiting = rights.waiting_since["a"];
        let resumed = rights.allocate(
            &BTreeMap::new(),
            &retained,
            &[
                request("a", &[(1, 0), (2, 0), (3, 0)]),
                request("b", &[(3, 0)]),
            ],
        );
        assert!(resumed.granted.contains("a"));
        assert_eq!(rights.waiting_since["a"], waiting);
        rights.confirm(&resumed.granted);
        assert!(!rights.waiting_since.contains_key("a"));
    }
}

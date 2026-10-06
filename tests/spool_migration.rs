use mapf_rl_simulator::spool::{DurableSpool, SpoolError};
use sha2::{Digest, Sha256};
use std::fs;
use tempfile::TempDir;
use uuid::Uuid;

// Exact v1 writer field order, independent of the migration implementation.
fn legacy_document(completed: &str) -> String {
    let completion_field = if completed.is_empty() {
        String::new()
    } else {
        format!(r#","completedOrders":{completed}"#)
    };
    let body = format!(
        r#"{{"formatVersion":1,"simulatorId":"sim-1","currentBootId":"{}","nextReportSequence":7,"pending":[],"deadLetters":[],"appliedOrder":null{completion_field}}}"#,
        Uuid::new_v4()
    );
    let checksum = format!("{:x}", Sha256::digest(body.as_bytes()));
    format!(r#"{{"body":{body},"checksumSha256":"{checksum}"}}"#)
}

#[test]
fn v1_array_map_and_empty_history_migrate_and_restart_with_terminal_fences() {
    for completed in [r#"["order-1"]"#, r#"{"order-1":410}"#, ""] {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("spool.json");
        let old = legacy_document(completed);
        fs::write(&path, &old).unwrap();
        let spool = DurableSpool::open(&path, "sim-1").unwrap();
        assert_eq!(spool.was_order_completed("order-1"), !completed.is_empty());
        assert_eq!(spool.was_completed("order-1", 411), !completed.is_empty());
        assert_eq!(
            fs::read_to_string(temp.path().join("spool.json.v1.bak")).unwrap(),
            old
        );
        let document: serde_json::Value =
            serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(document["body"]["formatVersion"], 2);
        drop(spool);
        let reopened = DurableSpool::open(&path, "sim-1").unwrap();
        assert_eq!(
            reopened.was_completed("order-1", u64::MAX),
            !completed.is_empty()
        );
        assert_eq!(
            fs::read_to_string(temp.path().join("spool.json.v1.bak")).unwrap(),
            old
        );
    }
}

#[test]
fn corrupt_v1_and_v2_files_are_rejected_without_rewriting_them() {
    for completed in [r#"["order-1"]"#, r#"{"order-1":410}"#] {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("spool.json");
        let old = legacy_document(completed);
        let corrupt = old.replace("order-1", "order-2");
        fs::write(&path, &corrupt).unwrap();
        assert!(matches!(
            DurableSpool::open(&path, "sim-1"),
            Err(SpoolError::Corrupt)
        ));
        assert_eq!(fs::read_to_string(&path).unwrap(), corrupt);
        assert!(!temp.path().join("spool.json.v1.bak").exists());

        fs::write(&path, old).unwrap();
        drop(DurableSpool::open(&path, "sim-1").unwrap());
        let corrupt = fs::read_to_string(&path)
            .unwrap()
            .replace("order-1", "order-2");
        fs::write(&path, &corrupt).unwrap();
        assert!(matches!(
            DurableSpool::open(&path, "sim-1"),
            Err(SpoolError::Corrupt)
        ));
        assert_eq!(fs::read_to_string(&path).unwrap(), corrupt);
    }
}

#[test]
fn migration_can_resume_after_backup_but_never_overwrites_another_backup() {
    for matching in [true, false] {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("spool.json");
        let backup = temp.path().join("spool.json.v1.bak");
        let old = legacy_document(r#"["order-1"]"#);
        let backup_bytes = if matching {
            old.as_str()
        } else {
            "other backup"
        };
        fs::write(&path, &old).unwrap();
        fs::write(&backup, backup_bytes).unwrap();
        assert_eq!(DurableSpool::open(&path, "sim-1").is_ok(), matching);
        assert_eq!(fs::read_to_string(&backup).unwrap(), backup_bytes);
        if !matching {
            assert_eq!(fs::read_to_string(&path).unwrap(), old);
        }
    }
}

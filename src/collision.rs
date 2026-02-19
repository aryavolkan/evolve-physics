use crate::BodyHandle;
use nalgebra::{Point2, Vector2};
use serde::{Deserialize, Serialize};

/// Event representing a collision between two bodies
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollisionEvent {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub contact_point: Point2<f32>,
    pub normal: Vector2<f32>,
    pub depth: f32,
    pub event_type: CollisionEventType,
}

/// Type of collision event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollisionEventType {
    /// Collision started
    Started,
    /// Collision ended
    Stopped,
}

/// Information about a contact between two bodies
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactPair {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub contacts: Vec<Contact>,
}

/// A single contact point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Contact {
    pub point: Point2<f32>,
    pub normal: Vector2<f32>,
    pub depth: f32,
}

impl CollisionEvent {
    /// Create a new collision started event
    pub fn started(
        body_a: BodyHandle,
        body_b: BodyHandle,
        contact_point: Point2<f32>,
        normal: Vector2<f32>,
        depth: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            contact_point,
            normal,
            depth,
            event_type: CollisionEventType::Started,
        }
    }

    /// Create a new collision stopped event
    pub fn stopped(body_a: BodyHandle, body_b: BodyHandle) -> Self {
        Self {
            body_a,
            body_b,
            contact_point: Point2::origin(),
            normal: Vector2::zeros(),
            depth: 0.0,
            event_type: CollisionEventType::Stopped,
        }
    }

    /// Check if this is a collision start event
    pub fn is_started(&self) -> bool {
        self.event_type == CollisionEventType::Started
    }

    /// Check if this is a collision stop event
    pub fn is_stopped(&self) -> bool {
        self.event_type == CollisionEventType::Stopped
    }
}

impl ContactPair {
    /// Create a new contact pair
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> Self {
        Self {
            body_a,
            body_b,
            contacts: Vec::new(),
        }
    }

    /// Add a contact point
    pub fn add_contact(&mut self, point: Point2<f32>, normal: Vector2<f32>, depth: f32) {
        self.contacts.push(Contact {
            point,
            normal,
            depth,
        });
    }

    /// Get the average contact point
    pub fn average_contact_point(&self) -> Option<Point2<f32>> {
        if self.contacts.is_empty() {
            return None;
        }

        let sum = self
            .contacts
            .iter()
            .map(|c| c.point.coords)
            .fold(Vector2::zeros(), |acc, p| acc + p);

        Some(Point2::from(sum / self.contacts.len() as f32))
    }

    /// Get the deepest contact
    pub fn deepest_contact(&self) -> Option<&Contact> {
        self.contacts
            .iter()
            .max_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BodyHandle;

    #[test]
    fn test_collision_event_creation() {
        let body_a = BodyHandle::new();
        let body_b = BodyHandle::new();

        let event = CollisionEvent::started(
            body_a,
            body_b,
            Point2::new(5.0, 10.0),
            Vector2::new(0.0, 1.0),
            0.5,
        );

        assert!(event.is_started());
        assert!(!event.is_stopped());
        assert_eq!(event.contact_point, Point2::new(5.0, 10.0));
        assert_eq!(event.depth, 0.5);
    }

    #[test]
    fn test_contact_pair() {
        let body_a = BodyHandle::new();
        let body_b = BodyHandle::new();

        let mut pair = ContactPair::new(body_a, body_b);

        pair.add_contact(Point2::new(0.0, 0.0), Vector2::new(0.0, 1.0), 1.0);
        pair.add_contact(Point2::new(2.0, 0.0), Vector2::new(0.0, 1.0), 0.5);
        pair.add_contact(Point2::new(4.0, 0.0), Vector2::new(0.0, 1.0), 2.0);

        let avg = pair.average_contact_point().unwrap();
        assert_eq!(avg, Point2::new(2.0, 0.0));

        let deepest = pair.deepest_contact().unwrap();
        assert_eq!(deepest.depth, 2.0);
        assert_eq!(deepest.point, Point2::new(4.0, 0.0));
    }
}

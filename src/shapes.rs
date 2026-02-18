use serde::{Serialize, Deserialize};

/// Geometric shapes for collision detection
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Shape {
    Circle(Circle),
    Rectangle(Rectangle),
}

/// Circle shape
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle {
    pub radius: f32,
}

impl Circle {
    /// Create a new circle with given radius
    pub fn new(radius: f32) -> Self {
        assert!(radius > 0.0, "Circle radius must be positive");
        Self { radius }
    }
}

/// Rectangle shape (axis-aligned)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Rectangle {
    pub half_width: f32,
    pub half_height: f32,
}

impl Rectangle {
    /// Create a new rectangle with given half-extents
    pub fn new(half_width: f32, half_height: f32) -> Self {
        assert!(half_width > 0.0, "Rectangle half_width must be positive");
        assert!(half_height > 0.0, "Rectangle half_height must be positive");
        Self { half_width, half_height }
    }
    
    /// Create a rectangle from full width and height
    pub fn from_size(width: f32, height: f32) -> Self {
        Self::new(width / 2.0, height / 2.0)
    }
    
    /// Get the full width
    pub fn width(&self) -> f32 {
        self.half_width * 2.0
    }
    
    /// Get the full height
    pub fn height(&self) -> f32 {
        self.half_height * 2.0
    }
}

impl Shape {
    /// Get the bounding box dimensions (width, height)
    pub fn bounding_box(&self) -> (f32, f32) {
        match self {
            Shape::Circle(circle) => {
                let diameter = circle.radius * 2.0;
                (diameter, diameter)
            }
            Shape::Rectangle(rect) => {
                (rect.width(), rect.height())
            }
        }
    }
    
    /// Estimate the area of the shape
    pub fn area(&self) -> f32 {
        match self {
            Shape::Circle(circle) => std::f32::consts::PI * circle.radius * circle.radius,
            Shape::Rectangle(rect) => rect.width() * rect.height(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_circle_creation() {
        let circle = Circle::new(5.0);
        assert_eq!(circle.radius, 5.0);
    }
    
    #[test]
    #[should_panic(expected = "Circle radius must be positive")]
    fn test_circle_invalid_radius() {
        Circle::new(-1.0);
    }
    
    #[test]
    fn test_rectangle_creation() {
        let rect = Rectangle::new(10.0, 5.0);
        assert_eq!(rect.half_width, 10.0);
        assert_eq!(rect.half_height, 5.0);
        assert_eq!(rect.width(), 20.0);
        assert_eq!(rect.height(), 10.0);
    }
    
    #[test]
    fn test_rectangle_from_size() {
        let rect = Rectangle::from_size(30.0, 20.0);
        assert_eq!(rect.half_width, 15.0);
        assert_eq!(rect.half_height, 10.0);
    }
    
    #[test]
    fn test_shape_bounding_box() {
        let circle = Shape::Circle(Circle::new(5.0));
        assert_eq!(circle.bounding_box(), (10.0, 10.0));
        
        let rect = Shape::Rectangle(Rectangle::new(15.0, 10.0));
        assert_eq!(rect.bounding_box(), (30.0, 20.0));
    }
    
    #[test]
    fn test_shape_area() {
        let circle = Shape::Circle(Circle::new(5.0));
        let circle_area = circle.area();
        assert!((circle_area - 78.53982).abs() < 0.001);
        
        let rect = Shape::Rectangle(Rectangle::from_size(10.0, 20.0));
        assert_eq!(rect.area(), 200.0);
    }
}
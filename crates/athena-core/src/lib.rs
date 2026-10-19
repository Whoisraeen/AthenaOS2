//! Allocation-free body, communication, reflex, and drive primitives.
//! Hardware deadlines and protection domains are platform responsibilities.
#![no_std]

pub mod bus;
pub mod drive;
pub mod model;
pub mod spinal;

pub use model::*;
pub use spinal::*;

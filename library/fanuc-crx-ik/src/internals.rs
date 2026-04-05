//! The internals module contains different chunks of the geometric method's implementation. They
//! are made publicly accessible in the main library for testing, debugging, and checking the
//! feasibility and assumptions of the method as new robots are added to the CRX product line.

pub mod reduced;
mod candidate_points;
mod extract_joints;


// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! G-code commands.
//!
//! Refernence:
//! - https://reprap.org/wiki/G-code
//! - https://marlinfw.org/meta/gcode/

/// A macro for easily defining commands with many optional fields
macro_rules! def_command {
    ($cmd_name:ident, $doc_str:literal $(, $field_name:ident : $field_type:ty)*) => {
        #[doc = $doc_str]
        #[derive(Debug, Default, Clone)]
        #[allow(dead_code, non_camel_case_types)]
        pub struct $cmd_name {
            $(
                pub $field_name: Option<$field_type>,
            )*
        }

        impl $cmd_name {
            #[allow(dead_code)]
            pub fn new() -> Self {
                Self {
                    $(
                        $field_name: None,
                    )*
                }
            }

            $(
                #[allow(dead_code)]
                pub fn $field_name(mut self, value: $field_type) -> Self {
                    self.$field_name = Some(value);
                    self
                }
            )*

            #[allow(dead_code)]
            pub(crate) fn parse_args(input: &str) -> nom::IResult<&str, Self> {
                use nom::character::complete::{alpha1, space0};
                use nom::{Err, error::{Error, ErrorKind}};
                #[allow(unused_imports)]
                use crate::gcode::parser::parse_float_arg;

                #[allow(unused_mut)]
                let mut cmd = Self::new();

                let mut rest = input;
                loop {
                    let (space_rest, _) = space0(rest)?;
                    rest = space_rest;

                    #[allow(unused_variables)]
                    if let Ok((alpha_rest, arg_name)) = alpha1::<_, (&str, ErrorKind)>(rest) {
                        rest = alpha_rest;

                        if false {
                            // This if-false is for generation of else if using macro
                            // See: https://stackoverflow.com/a/75637095
                        }
                        $(
                            else if arg_name == stringify!($field_name).to_uppercase() {
                                // TODO: support non-float args
                                let (arg_rest, arg) = parse_float_arg(rest)?;
                                rest = arg_rest;
                                cmd = cmd.$field_name(arg);
                            }
                        )*
                        else {
                            return nom::IResult::Err(Err::Error(Error::new(rest, ErrorKind::Char))) // TODO:
                        }
                    } else {
                        break
                    }
                }

                Ok((rest, cmd))
            }
        }

        impl std::fmt::Display for $cmd_name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}", stringify!($cmd_name))?;

                $(
                    if let Some(arg) = self.$field_name {
                        write!(f, " {}{:.5}", stringify!($field_name).to_uppercase(), arg)?;
                    }
                )*

                Ok(())
            }
        }
    }
}

def_command!(G0, "Rapid move", x: f64, y: f64, z: f64, a: f64, b: f64, c: f64, e: f64, f: f64);
def_command!(G1, "Linear move", x: f64, y: f64, z: f64, a: f64, b: f64, c: f64, e: f64, f: f64);
def_command!(G92, "Set position", x: f64, y: f64, z: f64, a: f64, b: f64, c: f64, e: f64);
// There is a 3D printer which use M1001 and M1002 to signal beginning and ending of start/end macros
//  https://www.ideamaker.io/dictionaryDetail.html?name=End%20of%20Start%20Gcode&category_name=Printer%20Settings
def_command!(BEGIN_DEWARP, "Enable dewarping", x: f64, y: f64);
def_command!(END_DEWARP, "Disable dewarping");

/// A single G-code command.
#[derive(Debug, Clone)]
pub enum Command {
    #[allow(dead_code)]
    G0(G0),
    #[allow(dead_code)]
    G1(G1),
    #[allow(dead_code)]
    G92(G92),
    #[allow(dead_code, non_camel_case_types)]
    BEGIN_DEWARP(BEGIN_DEWARP),
    #[allow(dead_code, non_camel_case_types)]
    END_DEWARP(END_DEWARP),
}

#[cfg(test)]
mod tests {
    def_command!(T0, "test", a: f64, b: f64);

    #[test]
    fn single_cmd_create_and_set() {
        let cmd = T0::new().b(123.0);

        assert_eq!(cmd.a, None);
        assert_eq!(cmd.b, Some(123.0));
    }

    #[test]
    fn parse_args() {
        let (_, cmd) = T0::parse_args("A1 B-2.3").unwrap();

        assert_eq!(cmd.a, Some(1.0));
        assert_eq!(cmd.b, Some(-2.3));
    }
}

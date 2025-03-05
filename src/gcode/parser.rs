// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use core::str::FromStr;
use nom::character::complete::{
    char, digit0, digit1, not_line_ending, satisfy, space0,
};
use nom::multi::many0;
use nom::IResult;
use nom::{
    branch::alt,
    combinator::{map, opt, recognize},
    sequence::{pair, preceded, separated_pair, terminated, tuple},
};
use nom::{
    error::{Error, ErrorKind},
    Err,
};

use super::command::{Command, BEGIN_DEWARP, END_DEWARP, G0, G1, G92};

pub(crate) fn parse_float_arg(input: &str) -> IResult<&str, f64> {
    map(
        recognize(tuple((
            opt(parse_sign),
            alt((
                recognize(separated_pair(digit0, char('.'), digit0)), // This should be tried first
                recognize(digit1),
            )),
        ))),
        |s| f64::from_str(s).unwrap(),
    )(input) // from_str will succeed
}

fn parse_sign(input: &str) -> IResult<&str, &str> {
    recognize(alt((char('+'), char('-'))))(input)
}

fn parse_cmd_name(input: &str) -> IResult<&str, &str> {
    recognize(pair(
        satisfy(|c| c.is_alphabetic() | (c == '_')),
        many0(satisfy(|c| c.is_alphanumeric() | (c == '_'))),
    ))(input)
}

fn parse_command(input: &str) -> IResult<&str, Command> {
    let (cmd_rest, cmd) = parse_cmd_name(input)?;

    match cmd {
        "G0" => map(G0::parse_args, Command::G0)(cmd_rest),
        "G1" => map(G1::parse_args, Command::G1)(cmd_rest),
        "G92" => map(G92::parse_args, Command::G92)(cmd_rest),
        "BEGIN_DEWARP" => map(BEGIN_DEWARP::parse_args, Command::BEGIN_DEWARP)(cmd_rest),
        "END_DEWARP" => map(END_DEWARP::parse_args, Command::END_DEWARP)(cmd_rest),
        _ => IResult::Err(Err::Error(Error::new(input, ErrorKind::Alpha))), // TODO:
    }
}

fn parse_line_num(input: &str) -> IResult<&str, u32> {
    preceded(char('N'), map(digit1, |s| u32::from_str(s).unwrap_or(0)))(input)
}

pub fn parse_line(input: &str) -> IResult<&str, Option<Command>> {
    let (rest, (_, cmd, _)) = tuple((
        opt(terminated(parse_line_num, space0)),
        opt(parse_command),
        opt(preceded(char(';'), not_line_ending)),
    ))(input)?;

    Ok((rest, cmd))
}

#[cfg(test)]
mod tests {
    use crate::gcode::command::Command;

    use super::{parse_float_arg, parse_line};

    #[test]
    fn float_arg() {
        let (_, arg) = parse_float_arg("1024").unwrap();
        assert_eq!(arg, 1024.0);

        let (_, arg) = parse_float_arg("-1.23").unwrap();
        assert_eq!(arg, -1.23);
    }

    #[test]
    fn line() {
        let (rest, opt_cmd) = parse_line("N1 G1 X1.2 Y-3.45; comment").unwrap();

        assert_eq!(rest, "");

        match opt_cmd {
            Some(Command::G1(cmd)) => {
                assert_eq!(cmd.x, Some(1.2));
                assert_eq!(cmd.y, Some(-3.45));
                assert_eq!(cmd.z, None);
            }
            _ => panic!(),
        }
    }
}

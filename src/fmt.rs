//! Internal logging facade.
//!
//! Each macro expands to a `defmt` call when the `defmt` feature is on, a `log`
//! call when the `log` feature is on, and to nothing (arguments are still
//! borrow-checked, so they can't rot) when neither is enabled. The two backends
//! are mutually exclusive.
//!
//! Usage matches `log` / `defmt`: `debug!("sync established, cps={}", cps)`.
//! Only a leading string literal followed by `{}`-style positional arguments is
//! supported - that's the common subset both backends accept.

#![allow(unused_macros)]

#[cfg(all(feature = "defmt", feature = "log"))]
compile_error!("`defmt` and `log` are mutually exclusive - enable at most one");

macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::trace!($s $(, $x)*);
        #[cfg(feature = "log")]
        ::log::trace!($s $(, $x)*);
        #[cfg(not(any(feature = "defmt", feature = "log")))]
        let _ = ($( &$x ),*);
    }};
}

macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::debug!($s $(, $x)*);
        #[cfg(feature = "log")]
        ::log::debug!($s $(, $x)*);
        #[cfg(not(any(feature = "defmt", feature = "log")))]
        let _ = ($( &$x ),*);
    }};
}

macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::info!($s $(, $x)*);
        #[cfg(feature = "log")]
        ::log::info!($s $(, $x)*);
        #[cfg(not(any(feature = "defmt", feature = "log")))]
        let _ = ($( &$x ),*);
    }};
}

macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::warn!($s $(, $x)*);
        #[cfg(feature = "log")]
        ::log::warn!($s $(, $x)*);
        #[cfg(not(any(feature = "defmt", feature = "log")))]
        let _ = ($( &$x ),*);
    }};
}

macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {{
        #[cfg(feature = "defmt")]
        ::defmt::error!($s $(, $x)*);
        #[cfg(feature = "log")]
        ::log::error!($s $(, $x)*);
        #[cfg(not(any(feature = "defmt", feature = "log")))]
        let _ = ($( &$x ),*);
    }};
}

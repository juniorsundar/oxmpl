# Rationale

OMPL is great, but it isn't written in Rust. While that's not a valid reason to rewrite it, the truth is C++ doesn't particularly *spark joy* for many modern developers.

The creator of OxMPL started this project while teaching themselves Rust. After a few small projects, they found that Rust *sparked joy*. To really dive deep into the language, they decided to rewrite something that already exists: OMPL.

## Goals

The goal isn't necessarily to create a full "drop-in" replacement for OMPL. Rust's traits and implementations can handle OMPL's modular nature more elegantly than C++'s inheritance models in some cases.

However, the project aims to:
1. Provide a memory-safe and high-performance core in Rust.
2. Offer Python and JavaScript bindings so the library can be used by researchers and web developers not keen on diving into Rust.
3. Mimic the high-level structure of OMPL to make it familiar and easy to adopt.

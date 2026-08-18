/// Which animation style is currently driving the LEDs.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Pattern {
    /// The two LED sets breathe up and down in alternation.
    #[default]
    AlternateBreathe,
    /// Both LED sets breathe up and down together.
    SyncBreathe,
    /// Both sides flicker independently at random.
    Twinkle,
    /// Both sides held at a fixed brightness, no animation.
    Solid,
}

impl Pattern {
    pub const ALL: [Pattern; 4] = [
        Pattern::AlternateBreathe,
        Pattern::SyncBreathe,
        Pattern::Twinkle,
        Pattern::Solid,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Pattern::AlternateBreathe => "alternate",
            Pattern::SyncBreathe => "sync",
            Pattern::Twinkle => "twinkle",
            Pattern::Solid => "solid",
        }
    }

    pub fn from_str(s: &str) -> Option<Pattern> {
        Pattern::ALL.into_iter().find(|p| p.as_str() == s)
    }

    pub fn to_code(self) -> u8 {
        match self {
            Pattern::AlternateBreathe => 0,
            Pattern::SyncBreathe => 1,
            Pattern::Twinkle => 2,
            Pattern::Solid => 3,
        }
    }

    pub fn from_code(code: u8) -> Pattern {
        match code {
            1 => Pattern::SyncBreathe,
            2 => Pattern::Twinkle,
            3 => Pattern::Solid,
            _ => Pattern::AlternateBreathe,
        }
    }
}

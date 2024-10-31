use anyhow::Result;
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::CrosstermBackend,
    prelude::{Backend, Constraint},
    style::{Color, Style},
    widgets::{Row, Table, TableState},
    Frame, Terminal,
};
use std::{
    fmt, io,
    ops::ControlFlow,
    time::{Duration, Instant},
};

#[derive(Debug, Clone)]
pub struct Page(pub Vec<Record>);

#[derive(Debug, Clone)]
pub struct Record(pub Vec<Value>);

#[derive(Debug, Clone)]
pub enum Value {
    I(i64),
    F(f64),
    S(String),
}

macro_rules! derive_from_trait_for_integer {
    ($ty:ty) => {
        impl From<$ty> for Value {
            fn from(value: $ty) -> Self {
                Self::I(value as i64)
            }
        }
    };
}

macro_rules! derive_from_trait_for_float {
    ($ty:ty) => {
        impl From<$ty> for Value {
            fn from(value: $ty) -> Self {
                Self::F(value as f64)
            }
        }
    };
}

derive_from_trait_for_integer!(i8);
derive_from_trait_for_integer!(i16);
derive_from_trait_for_integer!(i32);
derive_from_trait_for_integer!(i64);
derive_from_trait_for_integer!(isize);
derive_from_trait_for_integer!(u8);
derive_from_trait_for_integer!(u16);
derive_from_trait_for_integer!(u32);
derive_from_trait_for_integer!(u64);
derive_from_trait_for_integer!(usize);
derive_from_trait_for_float!(f32);
derive_from_trait_for_float!(f64);

impl From<String> for Value {
    fn from(value: String) -> Self {
        Self::S(value)
    }
}

impl From<&str> for Value {
    fn from(value: &str) -> Self {
        Self::S(value.to_string())
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::I(val) => write!(f, "{val}"),
            Value::F(val) => write!(f, "{val}"),
            Value::S(val) => write!(f, "{val}"),
        }
    }
}

pub fn run_tui(header: Vec<String>, data: Vec<Record>) -> Result<(), io::Error> {
    // setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    terminal.clear()?;

    let mut tui = Tui::new(10, header, data);
    tui.run_loop(&mut terminal)?;

    // restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    Ok(())
}

struct Tui {
    tick_dur: Duration,
    table_height: usize,
    table_state: TableState,
    n_records: usize,
    header: Row<'static>,
    rows: Vec<Row<'static>>,
    widths: Vec<Constraint>,
}

impl Tui {
    fn new(refresh_rate: u32, header: Vec<String>, data: Vec<Record>) -> Self {
        let tick_dur = Duration::from_secs(1) / refresh_rate;

        let n_records = data.len();
        let records: Vec<_> = data
            .iter()
            .map(|record| {
                let row: Vec<_> = record.0.iter().map(|val| format!("{val}")).collect();
                row
            })
            .collect();

        let widths: Vec<_> = header
            .iter()
            .enumerate()
            .map(|(idx, title)| {
                let max_len = records
                    .iter()
                    .map(|row| row[idx].len())
                    .max()
                    .unwrap_or(0)
                    .max(title.len());

                Constraint::Min(max_len as u16)
            })
            .collect();

        let header =
            Row::new(header.clone()).style(Style::default().fg(Color::Black).bg(Color::Green));
        let rows: Vec<_> = records.into_iter().map(Row::new).collect();

        let mut table_state = TableState::default();
        if n_records > 0 {
            table_state.select(Some(0));
        }

        Tui {
            table_state,
            tick_dur,
            table_height: 1,
            header,
            rows,
            widths,
            n_records,
        }
    }

    fn render<'a>(&mut self, frame: &mut Frame) {
        let Self {
            ref mut table_height,
            ref mut table_state,
            ref header,
            ref rows,
            ref widths,
            ..
        } = *self;

        let area = frame.area();
        *table_height = (area.height as usize).saturating_sub(3).max(1);

        let table = Table::new(rows.clone(), &self.widths)
            .header(header.clone())
            .widths(widths)
            .column_spacing(2)
            .row_highlight_style(Style::default().fg(Color::Black).bg(Color::White));

        frame.render_stateful_widget(table, area, table_state);
    }

    fn run_loop<'a, B: Backend + 'a>(&mut self, terminal: &mut Terminal<B>) -> io::Result<()> {
        let mut last_tick = Instant::now();

        loop {
            // Wait for key event
            {
                let timeout = self
                    .tick_dur
                    .checked_sub(last_tick.elapsed())
                    .unwrap_or_else(|| Duration::from_secs(0));

                // Process keyboard events
                let ctrl_flow = self.process_events(timeout)?;
                if let ControlFlow::Break(_) = ctrl_flow {
                    break;
                }
            }

            let elapsed_time = last_tick.elapsed();
            if elapsed_time >= self.tick_dur {
                // Draw UI
                terminal.draw(|frame| self.render(frame))?;

                // Clean up state
                last_tick = Instant::now();
            }
        }

        Ok(())
    }

    fn process_events(&mut self, timeout: Duration) -> io::Result<ControlFlow<()>> {
        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                use KeyCode as C;

                match key.code {
                    C::Char('q') => return Ok(ControlFlow::Break(())),
                    C::Up => {
                        self.key_up();
                    }
                    C::Down => {
                        self.key_down();
                    }
                    C::Left => {}
                    C::Right => {}
                    C::PageUp => {
                        self.key_page_up();
                    }
                    C::PageDown => {
                        self.key_page_down();
                    }
                    C::Home => {
                        self.key_home();
                    }
                    C::End => {
                        self.key_end();
                    }
                    _ => {}
                }
            }
        }

        Ok(ControlFlow::Continue(()))
    }

    fn key_up(&mut self) {
        if self.n_records > 0 {
            let new_idx = match self.table_state.selected() {
                Some(idx) => idx.saturating_sub(1),
                None => 0,
            };
            self.table_state.select(Some(new_idx));
        }
    }

    fn key_down(&mut self) {
        if let Some(last_idx) = self.n_records.checked_sub(1) {
            let new_idx = match self.table_state.selected() {
                Some(idx) => idx.saturating_add(1).min(last_idx),
                None => 0,
            };
            self.table_state.select(Some(new_idx));
        }
    }

    fn key_page_up(&mut self) {
        if self.n_records > 0 {
            let orig_idx = self.table_state.selected().unwrap_or(0);
            let new_idx = orig_idx.saturating_sub(self.table_height);
            self.table_state.select(Some(new_idx));
            *self.table_state.offset_mut() -= orig_idx - new_idx;
        }
    }

    fn key_page_down(&mut self) {
        if let Some(last_idx) = self.n_records.checked_sub(1) {
            let orig_idx = self.table_state.selected().unwrap_or(0);
            let new_idx = orig_idx.saturating_add(self.table_height).min(last_idx);
            self.table_state.select(Some(new_idx));
            *self.table_state.offset_mut() += new_idx - orig_idx;
        }
    }

    fn key_home(&mut self) {
        if self.n_records > 0 {
            self.table_state.select(Some(0));
        }
    }

    fn key_end(&mut self) {
        if let Some(idx) = self.n_records.checked_sub(1) {
            self.table_state.select(Some(idx));
        }
    }
}

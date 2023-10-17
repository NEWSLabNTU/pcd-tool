use crate::{opts::Dump, types::FileFormat, utils::guess_file_format};
use anyhow::{anyhow, Result};
use itertools::Itertools;
use std::path::Path;

use self::tui::run_tui;

mod tui {
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
        io,
        ops::ControlFlow,
        time::{Duration, Instant},
    };

    pub fn run_tui(header: Vec<String>, data: Vec<Vec<f32>>) -> Result<(), io::Error> {
        // setup terminal
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        terminal.clear()?;

        let tick_dur = Duration::from_secs(1) / 10;
        let mut tui = Tui {
            table_state: TableState::default(),
            header,
            data,
            tick_dur,
            table_height: 1,
        };

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
        header: Vec<String>,
        data: Vec<Vec<f32>>,
    }

    impl Tui {
        fn render<'a, B: Backend + 'a>(&mut self, frame: &mut Frame<B>) {
            let area = frame.size();
            self.table_height = (area.height as usize).saturating_sub(3).max(1);

            let rows: Vec<_> = self
                .data
                .iter()
                .map(|row| {
                    let row: Vec<_> = row.iter().map(|&val| format!("{val}")).collect();
                    row
                })
                .collect();

            let widths: Vec<_> = self
                .header
                .iter()
                .enumerate()
                .map(|(idx, title)| {
                    let max_len = rows
                        .iter()
                        .map(|row| row[idx].len())
                        .max()
                        .unwrap_or(0)
                        .max(title.len());

                    Constraint::Min(max_len as u16)
                })
                .collect();

            let header = Row::new(self.header.clone())
                .style(Style::default().fg(Color::Black).bg(Color::Green));
            let rows: Vec<_> = rows.into_iter().map(Row::new).collect();

            let table = Table::new(rows)
                .header(header)
                .widths(&widths)
                .column_spacing(1)
                .highlight_style(Style::default().fg(Color::Black).bg(Color::White));

            frame.render_stateful_widget(table, area, &mut self.table_state);
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
            if !self.data.is_empty() {
                let new_idx = match self.table_state.selected() {
                    Some(idx) => idx.saturating_sub(1),
                    None => 0,
                };
                self.table_state.select(Some(new_idx));
            }
        }

        fn key_down(&mut self) {
            if let Some(last_idx) = self.data.len().checked_sub(1) {
                let new_idx = match self.table_state.selected() {
                    Some(idx) => idx.saturating_add(1).min(last_idx),
                    None => 0,
                };
                self.table_state.select(Some(new_idx));
            }
        }

        fn key_page_up(&mut self) {
            if !self.data.is_empty() {
                let orig_idx = self.table_state.selected().unwrap_or(0);
                let new_idx = orig_idx.saturating_sub(self.table_height);
                self.table_state.select(Some(new_idx));
                *self.table_state.offset_mut() -= orig_idx - new_idx;
            }
        }

        fn key_page_down(&mut self) {
            if let Some(last_idx) = self.data.len().checked_sub(1) {
                let orig_idx = self.table_state.selected().unwrap_or(0);
                let new_idx = orig_idx.saturating_add(self.table_height).min(last_idx);
                self.table_state.select(Some(new_idx));
                *self.table_state.offset_mut() += new_idx - orig_idx;
            }
        }

        fn key_home(&mut self) {
            if !self.data.is_empty() {
                self.table_state.select(Some(0));
            }
        }

        fn key_end(&mut self) {
            if let Some(idx) = self.data.len().checked_sub(1) {
                self.table_state.select(Some(idx));
            }
        }
    }
}

pub fn dump(args: Dump) -> Result<()> {
    let Dump { input, format } = args;

    let format = match format {
        Some(format) => format,
        None => guess_file_format(&input)
            .ok_or_else(|| anyhow!("unable to guess file format of '{}'", input.display()))?,
    };

    use FileFormat as F;
    match format {
        F::LibpclPcd | F::NewslabPcd => dump_pcd(&input)?,
        F::VelodynePcap => dump_velodyne_pcap(&input)?,
    }

    Ok(())
}

fn dump_pcd<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    let reader = pcd_rs::DynReader::open(path)?;

    let header = vec!["x".to_string(), "y".to_string(), "z".to_string()];
    let data: Vec<_> = reader
        .map(|record| -> Result<_> {
            let record = record?;
            let [x, y, z]: [f32; 3] = record.to_xyz().unwrap();
            Ok(vec![x, y, z])
        })
        .try_collect()?;

    run_tui(header, data)?;
    Ok(())
}

fn dump_velodyne_pcap<P>(path: P) -> Result<()>
where
    P: AsRef<Path>,
{
    todo!();
}

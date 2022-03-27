from rich.traceback import install
from rich.console import Group
from rich.panel import Panel
from rich.live import Live
from rich.table import Column
from rich.progress import (
    Progress,
    TimeElapsedColumn,
    BarColumn,
    SpinnerColumn,
    TextColumn,
)
install()


class KinematicsProgress(Live):
    def __init__(self, total_steps=10, start_states=[0, 0, 0, 0]) -> None:
        self.simulation_progress = Progress(
            TimeElapsedColumn(),
            TextColumn("Running Simulation"),
            BarColumn(),
            SpinnerColumn(),
            TextColumn("{task.percentage:>3.0f}%"),
        )
        self.state_progress = Progress(
            TextColumn(
                "[bold green]Ground Truth:[not bold]\n{task.fields[ground]}",
                table_column=Column(ratio=1),
            ),
            TextColumn(
                "[bold cyan]Noisy Rover:[not bold]\n{task.fields[noisy]}",
                table_column=Column(ratio=1),
            ),
            expand=True,
        )

        self.state_task = self.state_progress.add_task(
            "", ground=start_states, noisy=start_states
        )

        self.sim_task = self.simulation_progress.add_task(
            "Running Simulation!", total=total_steps
        )

        self.render_group = Panel.fit(
            Group(self.state_progress, self.simulation_progress),
            title="Simulation Progress",
        )
        super().__init__(self.render_group)

    def step(self, ground_truth, noisy_state):
        self.simulation_progress.advance(self.sim_task)
        self.state_progress.update(
            self.state_task, ground=ground_truth, noisy=noisy_state
        )

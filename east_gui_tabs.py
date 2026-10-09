"""Output tabs with stable visibility during rapid selection changes."""
import customtkinter as ctk


class OutputTabs(ctk.CTkTabview):
    def _grid_forget_all_tabs(self, exclude_name=None):
        # CTk 5.2.2 schedules old selections for removal 100 ms later. An older
        # callback must not hide the currently selected tab after another switch.
        if exclude_name is not None:
            exclude_name = self.get()
        super()._grid_forget_all_tabs(exclude_name=exclude_name)

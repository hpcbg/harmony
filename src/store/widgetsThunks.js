import { createAsyncThunk } from "@reduxjs/toolkit";
import api from "../services/api";

export const runButtonWidget = createAsyncThunk(
  "widgets/runButton",
  async (widget) => {
    // Example async behavior
    await new Promise((resolve) => setTimeout(resolve, 1000));
    await api.put(widget.endpoint, { action: widget.name });
    console.log("Button widget executed:", widget.id);

    return widget.id;
  }
);

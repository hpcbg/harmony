import { createSlice, createAsyncThunk } from "@reduxjs/toolkit";
import api from "../services/api";

export const fetchWidgetData = createAsyncThunk(
  "widgetData/fetch",
  async ({ widgetId, endpoint }, { rejectWithValue }) => {
    try {
      const data = await api.get(endpoint);
      return { widgetId, data };
    } catch (err) {
      return rejectWithValue(err.message);
    }
  }
);

const widgetDataSlice = createSlice({
  name: "widgetData",
  initialState: {
    values: {}, // widgetId -> data
    loading: {}, // widgetId -> boolean
    error: {}, // widgetId -> error
  },
  reducers: {
    setWidgetData(state, action) {
      const { widgetId, data } = action.payload;
      state.values[widgetId] = data;
    },
    clearWidgetData(state, action) {
      delete state.values[action.payload];
      delete state.loading[action.payload];
      delete state.error[action.payload];
    },
  },
  extraReducers: (builder) => {
    builder
      .addCase(fetchWidgetData.pending, (state, action) => {
        state.loading[action.meta.arg.widgetId] = true;
      })
      .addCase(fetchWidgetData.fulfilled, (state, action) => {
        const { widgetId, data } = action.payload;
        state.loading[widgetId] = false;
        state.values[widgetId] = data;
      })
      .addCase(fetchWidgetData.rejected, (state, action) => {
        const widgetId = action.meta.arg.widgetId;
        state.loading[widgetId] = false;
        state.error[widgetId] = action.payload;
      });
  },
});

export const { clearWidgetData, setWidgetData } = widgetDataSlice.actions;
export default widgetDataSlice.reducer;

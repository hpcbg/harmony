import { createSlice, nanoid } from "@reduxjs/toolkit";
import { removeWidgetFromPage } from "./pagesSlice";

const initialState = {
  widgets: {},
  editingWidgetId: null,
  creatingNewWidget: false,
};

const widgetsSlice = createSlice({
  name: "widgets",
  initialState,
  reducers: {
    createWidget: {
      reducer(state, action) {
        const widget = action.payload;
        state.widgets[widget.id] = widget;
        state.creatingNewWidget = false;
      },
      prepare({ name, type, updateInterval, endpoint, maxValues }) {
        return {
          payload: {
            id: nanoid(),
            name: name,
            type: type,
            updateInterval: updateInterval,
            endpoint: endpoint,
            maxValues: maxValues,
          },
        };
      },
    },
    addWidget: (state, action) => {
      const widget = action.payload;
      state.widgets[widget.id] = widget;
      state.creatingNewWidget = false;
    },
    setWidgetsFromConfig: (state, action) => {
      state.widgets = action.payload;
      state.editingWidgetId = null;
      state.creatingNewWidget = false;
    },
    startCreateWidget(state) {
      state.creatingNewWidget = true;
      state.editingWidgetId = null;
    },
    updateWidget: (state, action) => {
      const { id, ...data } = action.payload;
      if (state.widgets[id]) {
        state.widgets[id] = {
          ...state.widgets[id],
          ...data,
        };
        state.editingWidgetId = null;
      }
    },
    deleteWidget: (state, action) => {
      const widgetId = action.payload;
      delete state.widgets[widgetId];
    },
    startEditWidget(state, action) {
      state.editingWidgetId = action.payload;
      state.creatingNewWidget = false;
    },
    stopEditWidget(state) {
      state.editingWidgetId = null;
      state.creatingNewWidget = false;
    },
  },
});

// Thunk to delete widget and remove from all pages
export const deleteWidgetAndRemoveFromPages =
  (widgetId) => (dispatch, getState) => {
    const state = getState();
    const pages = state.pages.pages;

    // Remove widget from all pages
    pages.forEach((page) => {
      if (page.widgets[widgetId]) {
        dispatch(
          removeWidgetFromPage({
            pageId: page.id,
            widgetId,
          }),
        );
      }
    });

    // Remove widget from widgets slice
    dispatch(deleteWidget(widgetId));
  };

export const {
  createWidget,
  addWidget,
  setWidgetsFromConfig,
  startCreateWidget,
  updateWidget,
  deleteWidget,
  removeWidget,
  startEditWidget,
  stopEditWidget,
} = widgetsSlice.actions;

export default widgetsSlice.reducer;

export const selectWidgets = (state) => state.widgets;

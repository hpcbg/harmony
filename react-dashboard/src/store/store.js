import { configureStore } from "@reduxjs/toolkit";

import pagesSlice from "./pagesSlice";
import widgetsSlice from "./widgetsSlice";
import widgetDataReducer from "./widgetDataSlice";

export const store = configureStore({
  reducer: {
    pages: pagesSlice,
    widgets: widgetsSlice,
    widgetData: widgetDataReducer,
  },
});

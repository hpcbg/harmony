import { configureStore, combineReducers } from "@reduxjs/toolkit";
import pagesReducer from "./pagesSlice";
import widgetsReducer from "./widgetsSlice";
import widgetDataReducer from "./widgetDataSlice";
import {
  persistStore,
  persistReducer,
  FLUSH,
  REHYDRATE,
  PAUSE,
  PERSIST,
  PURGE,
  REGISTER,
} from "redux-persist";
import storage from "redux-persist/lib/storage";

// Choose which slices to persist
const persistConfig = {
  key: "root",
  storage,
  whitelist: ["pages", "widgets"], // persist only these slices
};

// Combine reducers into one persisted reducer
const rootReducer = {
  pages: pagesReducer,
  widgets: widgetsReducer,
  widgetData: widgetDataReducer,
};

const persistedReducer = persistReducer(
  persistConfig,
  combineReducers(rootReducer),
);

export const store = configureStore({
  reducer: persistedReducer,
  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware({
      serializableCheck: {
        ignoredActions: [FLUSH, REHYDRATE, PAUSE, PERSIST, PURGE, REGISTER],
      },
    }),
});

export const persistor = persistStore(store);

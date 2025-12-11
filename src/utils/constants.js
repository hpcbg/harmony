// Widget types
export const WIDGET_TYPES = {
  NUMBER: "number",
  STATE: "state",
  HISTORY: "history",
  BUTTON: "button",
};

// Default widget settings
export const DEFAULT_WIDGET_CONFIG = {
  name: "",
  type: WIDGET_TYPES.NUMBER,
  updateInterval: 5000,
  maxValues: 20,
  endpoint: "/api/data/number",
};

// Default widget position
export const DEFAULT_WIDGET_POSITION = {
  x: 50,
  y: 50,
  width: 300,
  height: 200,
};

// Widget type configurations
export const WIDGET_TYPE_CONFIG = {
  [WIDGET_TYPES.NUMBER]: {
    label: "Number",
    description: "Displays a numeric value from the API",
    color: "blue",
    defaultEndpoint: "/api/data/number",
  },
  [WIDGET_TYPES.STATE]: {
    label: "State (Boolean)",
    description: "Shows active/inactive status with visual indicator",
    color: "green",
    defaultEndpoint: "/api/data/state",
  },
  [WIDGET_TYPES.HISTORY]: {
    label: "History (Graph)",
    description: "Visualizes historical data as a bar chart",
    color: "purple",
    defaultEndpoint: "/api/data/history",
  },
  [WIDGET_TYPES.BUTTON]: {
    label: "Button (Action)",
    description: "Sends PUT requests to trigger actions",
    color: "orange",
    defaultEndpoint: "/api/action",
  },
};

// LocalStorage keys
export const STORAGE_KEYS = {
  PAGES: "dashboard-pages",
  WIDGET_LIBRARY: "dashboard-widget-library",
  CURRENT_PAGE: "dashboard-current-page",
  EDIT_MODE: "dashboard-edit-mode",
};

// App settings
export const APP_CONFIG = {
  MIN_UPDATE_INTERVAL: 1000,
  MAX_UPDATE_INTERVAL: 60000,
  MIN_HISTORY_VALUES: 5,
  MAX_HISTORY_VALUES: 100,
  MIN_WIDGET_WIDTH: 200,
  MIN_WIDGET_HEIGHT: 150,
};

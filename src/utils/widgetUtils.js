/**
 * Generate a unique ID
 * @returns {string} Unique identifier
 */
export const generateId = () => {
  return Date.now().toString() + Math.random().toString(36).substr(2, 9);
};

/**
 * Validate widget configuration
 * @param {object} widget - Widget configuration
 * @returns {object} Validation result
 */
export const validateWidget = (widget) => {
  const errors = [];

  if (!widget.name || widget.name.trim() === "") {
    errors.push("Widget name is required");
  }

  if (!widget.type) {
    errors.push("Widget type is required");
  }

  if (widget.updateInterval < 1000) {
    errors.push("Update interval must be at least 1000ms");
  }

  if (
    widget.type === "history" &&
    (!widget.maxValues || widget.maxValues < 5)
  ) {
    errors.push("History widget must have at least 5 max values");
  }

  if (!widget.endpoint || widget.endpoint.trim() === "") {
    errors.push("API endpoint is required");
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

/**
 * Get widget type info
 * @param {string} type - Widget type
 * @returns {object} Widget type configuration
 */
export const getWidgetTypeInfo = (type) => {
  const config = {
    number: {
      label: "Number",
      color: "blue",
      bgColor: "bg-blue-50",
    },
    state: {
      label: "State",
      color: "green",
      bgColor: "bg-green-50",
    },
    history: {
      label: "History",
      color: "purple",
      bgColor: "bg-purple-50",
    },
    button: {
      label: "Button",
      color: "orange",
      bgColor: "bg-orange-50",
    },
  };

  return config[type] || config.number;
};

/**
 * Format update interval for display
 * @param {number} ms - Milliseconds
 * @returns {string} Formatted string
 */
export const formatUpdateInterval = (ms) => {
  if (ms < 1000) return `${ms}ms`;
  if (ms < 60000) return `${ms / 1000}s`;
  return `${ms / 60000}m`;
};

/**
 * Check if widget is used on any page
 * @param {string} widgetId - Widget ID
 * @param {array} pages - All pages
 * @returns {boolean} True if widget is used
 */
export const isWidgetUsed = (widgetId, pages) => {
  return pages.some((page) => page.widgets.some((w) => w.id === widgetId));
};

/**
 * Get pages where widget is used
 * @param {string} widgetId - Widget ID
 * @param {array} pages - All pages
 * @returns {array} Array of page names
 */
export const getWidgetPages = (widgetId, pages) => {
  return pages
    .filter((page) => page.widgets.some((w) => w.id === widgetId))
    .map((page) => page.name);
};

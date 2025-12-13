import { useState, useEffect, useRef } from "react";
import API from "../services/api";

const useWidgetData = (widgets, isActive = true) => {
  const [widgetData, setWidgetData] = useState({});
  const intervalsRef = useRef({});
  const historyDataRef = useRef({}); // Store history data persistently

  useEffect(() => {
    // Don't fetch data if widgets aren't active (not on current page)
    if (!isActive || !widgets || widgets.length === 0) {
      return;
    }

    const fetchWidgetData = async (widget) => {
      // Skip button widgets as they don't fetch data
      if (widget.type === "button") return;

      try {
        const data = await API.get(widget.endpoint);

        // Special handling for history widgets
        if (widget.type === "history") {
          // Initialize history array if it doesn't exist
          if (!historyDataRef.current[widget.id]) {
            historyDataRef.current[widget.id] = [];
          }

          // Get the new value from the response
          const newValue =
            data?.value !== undefined
              ? data.value
              : data?.values?.[0] !== undefined
              ? data.values[0]
              : null;

          if (newValue !== null) {
            // Add new value to history
            historyDataRef.current[widget.id].push(newValue);

            // Keep only the last maxValues entries
            const maxValues = widget.maxValues || 20;
            if (historyDataRef.current[widget.id].length > maxValues) {
              historyDataRef.current[widget.id] = historyDataRef.current[
                widget.id
              ].slice(-maxValues);
            }

            // Update widget data with accumulated history
            setWidgetData((prev) => ({
              ...prev,
              [widget.id]: {
                values: [...historyDataRef.current[widget.id]],
              },
            }));
          }
        } else {
          setWidgetData((prev) => ({ ...prev, [widget.id]: data }));
        }
      } catch (error) {
        console.error(`Error fetching data for widget ${widget.id}:`, error);
      }
    };

    // Clear existing intervals first
    Object.values(intervalsRef.current).forEach(clearInterval);
    intervalsRef.current = {};

    widgets.forEach((widget) => {
      // Initial fetch
      fetchWidgetData(widget);

      // Set up interval for periodic updates
      intervalsRef.current[widget.id] = setInterval(() => {
        fetchWidgetData(widget);
      }, widget.updateInterval);
    });

    // Cleanup intervals on unmount or when widgets change
    return () => {
      Object.values(intervalsRef.current).forEach(clearInterval);
      intervalsRef.current = {};
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [widgets.map((w) => w.id).join(","), isActive]); // Only re-run when active state, widget count, or widget IDs change

  // Clear history when a widget is removed
  useEffect(() => {
    const currentWidgetIds = new Set(widgets.map((w) => w.id));

    // Remove history for widgets that no longer exist
    Object.keys(historyDataRef.current).forEach((widgetId) => {
      if (!currentWidgetIds.has(widgetId)) {
        delete historyDataRef.current[widgetId];
      }
    });
  }, [widgets]);
  return widgetData;
};

export default useWidgetData;

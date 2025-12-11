import { useState, useEffect } from "react";
import API from "../services/api";

const useWidgetData = (widgets) => {
  const [widgetData, setWidgetData] = useState({});

  useEffect(() => {
    const intervals = {};

    const fetchWidgetData = async (widget) => {
      // Skip button widgets as they don't fetch data
      if (widget.type === "button") return;

      try {
        const data = await API.get(widget.endpoint);
        setWidgetData((prev) => ({ ...prev, [widget.id]: data }));
      } catch (error) {
        console.error(`Error fetching data for widget ${widget.id}:`, error);
      }
    };

    // Set up data fetching for each widget
    widgets.forEach((widget) => {
      // Initial fetch
      fetchWidgetData(widget);

      // Set up interval for periodic updates
      intervals[widget.id] = setInterval(() => {
        fetchWidgetData(widget);
      }, widget.updateInterval);
    });

    // Cleanup intervals on unmount or when widgets change
    return () => {
      Object.values(intervals).forEach(clearInterval);
    };
  }, [widgets]);

  return widgetData;
};

export default useWidgetData;

import { useState, useEffect, useRef } from "react";
import API from "../services/api";

const useWidgetData = (widgets, isActive = true) => {
  const [widgetData, setWidgetData] = useState({});
  const intervalsRef = useRef({});

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
        setWidgetData((prev) => ({ ...prev, [widget.id]: data }));
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

  return widgetData;
};

export default useWidgetData;

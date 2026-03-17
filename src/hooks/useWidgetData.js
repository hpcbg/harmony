import { useEffect, useRef } from "react";
import { useDispatch } from "react-redux";
import API from "../services/api";
import { setWidgetData, clearWidgetData } from "../store/widgetDataSlice";

const useWidgetData = (widgets, isActive) => {
  const dispatch = useDispatch();

  const intervalsRef = useRef({});
  const historyRef = useRef({});

  useEffect(() => {
    if (!isActive || widgets.length === 0) return;

    const validWidgets = widgets.filter((w) => w && w.id);

    const fetchWidgetData = async (widget) => {
      if (widget.type === "button") return;

      try {
        const data = await API.get(widget.endpoint);

        // HISTORY widgets
        if (
          widget.type === "historyLineChart" ||
          widget.type === "historyBarChart"
        ) {
          if (!historyRef.current[widget.id]) {
            historyRef.current[widget.id] = [];
          }

          const newValue = data?.value ?? data?.values?.[0] ?? null;

          if (newValue !== null) {
            historyRef.current[widget.id].push(newValue);

            const max = widget.maxValues || 20;
            historyRef.current[widget.id] =
              historyRef.current[widget.id].slice(-max);

            dispatch(
              setWidgetData({
                widgetId: widget.id,
                data: {
                  values: [...historyRef.current[widget.id]],
                },
              }),
            );
          }
        } else {
          dispatch(
            setWidgetData({
              widgetId: widget.id,
              data,
            }),
          );
        }
      } catch (err) {
        console.error("Widget fetch error:", err);
      }
    };

    // Clear existing intervals
    Object.values(intervalsRef.current).forEach(clearInterval);
    intervalsRef.current = {};

    validWidgets.forEach((widget) => {
      fetchWidgetData(widget);
      intervalsRef.current[widget.id] = setInterval(
        () => fetchWidgetData(widget),
        widget.updateInterval,
      );
    });

    return () => {
      Object.values(intervalsRef.current).forEach(clearInterval);
      intervalsRef.current = {};
    };
  }, [isActive, widgets.map((w) => w.id).join(",")]);

  // Cleanup removed widgets
  useEffect(() => {
    const ids = new Set(widgets.map((w) => w.id));

    Object.keys(historyRef.current).forEach((id) => {
      if (!ids.has(id)) {
        delete historyRef.current[id];
        dispatch(clearWidgetData(id));
      }
    });
  }, [widgets]);
};

export default useWidgetData;

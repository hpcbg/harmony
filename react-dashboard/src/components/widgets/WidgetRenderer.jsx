import ButtonWidget from "./types/ButtonWidget";
import NumberWidget from "./types/NumberWidget";
import StateWidget from "./types/StateWidget";
import ImageWidget from "./types/ImageWidget";
import HistoryWidgetBarChart from "./types/HistoryWidgetBarChart";
import HistoryWidgetLineChart from "./types/HistoryWidgetLineChart";

/**
 * WidgetRenderer dynamically renders the appropriate widget component
 * based on the widget type stored in Redux.
 *
 * @param {object} widget - The widget object from Redux
 */
export default function WidgetRenderer({ widget }) {
  if (!widget || !widget.type) return null;

  switch (widget.type) {
    case "button":
      return <ButtonWidget widget={widget} />;
    case "number":
      return <NumberWidget widget={widget} />;
    case "state":
      return <StateWidget widget={widget} />;
    case "image":
      return <ImageWidget widget={widget} />;
    case "historyBarChart":
      return <HistoryWidgetBarChart widget={widget} />;
    case "historyLineChart":
      return <HistoryWidgetLineChart widget={widget} />;
    default:
      return (
        <div className="h-full flex items-center justify-center text-gray-400">
          Unknown widget type: {widget.type}
        </div>
      );
  }
}

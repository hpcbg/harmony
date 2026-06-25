import { useSelector } from "react-redux";
import DraggableWidget from "../components/widgets/DraggableWidget";
import useWidgetData from "../hooks/useWidgetData";
import { selectPages } from "../store/pagesSlice.js";
import { selectWidgets } from "../store/widgetsSlice.js";

export default function DashboardPage() {
  const { currentPage, pages, editMode } = useSelector(selectPages);
  const currentPageObj = pages.find((p) => p.id === currentPage);
  const pageWidgets = currentPageObj.widgets ?? {};
  const widgets = useSelector(selectWidgets);
  // Poll widget data for current page
  useWidgetData(
    Object.keys(pageWidgets)
      .map((id) => widgets.widgets[id])
      .filter(Boolean),
    true,
  );

  return (
    <div className="w-full overflow-y-auto h-full relative bg-gray-50">
      {/* Render widgets */}
      {Object.keys(pageWidgets).length > 0 ? (
        Object.entries(pageWidgets).map(([widgetId, position]) => {
          const widget = widgets.widgets[widgetId];
          if (!widget) return null;
          return (
            <DraggableWidget
              key={widget.id}
              widget={widget}
              position={position}
            />
          );
        })
      ) : (
        // Empty state
        <div className="h-full flex items-center justify-center">
          <div className="text-center text-gray-400">
            <p className="text-lg mb-2">No widgets on this page yet</p>
            {editMode && (
              <p className="text-sm">Click "New Widget" to get started</p>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

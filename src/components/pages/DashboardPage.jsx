import { DraggableWidget } from "../widgets";

const DashboardPage = ({
  pageData,
  widgets,
  widgetData,
  isEditMode,
  onMoveWidget,
  onResizeWidget,
  onRemoveWidget,
  onEditWidget,
  renderWidget,
}) => {
  return (
    <div className="w-full h-full relative bg-gray-50">
      {pageData?.widgets.map((widgetPos, index) => {
        const widget = widgets.find((w) => w.id === widgetPos.id);
        if (!widget) return null;

        return (
          <DraggableWidget
            key={`${index}-${widget.id}`}
            widget={widget}
            position={widgetPos}
            onMove={onMoveWidget}
            onResize={onResizeWidget}
            onRemove={onRemoveWidget}
            onEdit={onEditWidget}
            isEditMode={isEditMode}
          >
            {renderWidget(widget, widgetData[widget.id])}
          </DraggableWidget>
        );
      })}

      {/* Empty state */}
      {(!pageData?.widgets || pageData.widgets.length === 0) && (
        <div className="h-full flex items-center justify-center">
          <div className="text-center text-gray-400">
            <p className="text-lg mb-2">No widgets on this page yet</p>
            {isEditMode && (
              <p className="text-sm">Click "New Widget" to get started</p>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default DashboardPage;

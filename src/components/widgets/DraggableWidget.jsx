import { useState, useEffect, useRef } from "react";
import { X, Edit, Menu } from "lucide-react";

// Draggable Widget Container
const DraggableWidget = ({
  widget,
  position,
  onMove,
  onResize,
  onRemove,
  onEdit,
  isEditMode,
  children,
}) => {
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [resizeStart, setResizeStart] = useState({
    x: 0,
    y: 0,
    width: 0,
    height: 0,
  });
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const ref = useRef(null);

  const handleMouseDown = (e) => {
    if (!isEditMode) return;
    if (e.target.closest(".widget-content")) return;
    if (e.target.closest(".widget-controls")) return;
    if (e.target.closest(".resize-handle")) return;

    // Prevent text selection while dragging
    e.preventDefault();

    setIsDragging(true);
    setDragStart({
      x: e.clientX - position.x,
      y: e.clientY - position.y,
    });
  };

  const handleResizeMouseDown = (e) => {
    if (!isEditMode) return;
    e.preventDefault();
    e.stopPropagation();

    setIsResizing(true);
    setResizeStart({
      x: e.clientX,
      y: e.clientY,
      width: position.width || 300,
      height: position.height || 200,
    });
  };

  useEffect(() => {
    if (!isDragging && !isResizing) return;

    const handleMouseMove = (e) => {
      if (isDragging) {
        onMove(widget.id, {
          x: e.clientX - dragStart.x,
          y: e.clientY - dragStart.y,
        });
      } else if (isResizing) {
        onResize(widget.id, {
          width: resizeStart.width + e.clientX - resizeStart.x,
          height: resizeStart.height + e.clientY - resizeStart.y,
        });
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
      setIsResizing(false);
    };

    document.addEventListener("mousemove", handleMouseMove);
    document.addEventListener("mouseup", handleMouseUp);

    return () => {
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("mouseup", handleMouseUp);
    };
  }, [
    isDragging,
    isResizing,
    dragStart,
    resizeStart,
    widget.id,
    onMove,
    onResize,
  ]);

  return (
    <div
      ref={ref}
      className={`absolute bg-white rounded-lg shadow-lg ${
        isEditMode ? (isDragging ? "cursor-grabbing" : "cursor-grab") : ""
      }`}
      style={{
        left: position.x,
        top: position.y,
        width: position.width || 300,
        height: position.height || 200,
      }}
      onMouseDown={handleMouseDown}
    >
      {isEditMode && (
        <>
          <div className="absolute top-2 left-2 text-gray-400">
            <Menu size={16} />
          </div>
          <div className="widget-controls absolute top-2 right-2 flex gap-1 z-10">
            <button
              onClick={() => onEdit(widget, true)}
              className="p-1 text-blue-500 hover:text-blue-700 bg-white rounded"
            >
              <Edit size={16} />
            </button>
            <button
              onClick={() => onRemove(widget.id)}
              className="p-1 text-red-500 hover:text-red-700 bg-white rounded"
            >
              <X size={16} />
            </button>
          </div>

          {/* Resize handle */}
          <div
            className="resize-handle absolute bottom-0 right-0 w-4 h-4 cursor-se-resize"
            onMouseDown={handleResizeMouseDown}
            title="Resize widget"
          >
            <svg
              className="absolute bottom-1 right-1 text-gray-400"
              width="12"
              height="12"
              viewBox="0 0 12 12"
            >
              <path
                d="M 10 2 L 2 10 M 10 6 L 6 10 M 10 10 L 10 10"
                stroke="currentColor"
                strokeWidth="1.5"
                fill="none"
              />
            </svg>
          </div>
        </>
      )}
      <div className="widget-content w-full h-full p-8">{children}</div>
    </div>
  );
};

export default DraggableWidget;

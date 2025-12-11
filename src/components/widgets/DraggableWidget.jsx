import { useState, useEffect, useRef } from "react";
import { X, Edit, Menu } from "lucide-react";

// Draggable Widget Container
const DraggableWidget = ({
  widget,
  position,
  onMove,
  onRemove,
  onEdit,
  isEditMode,
  children,
}) => {
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const ref = useRef(null);

  const handleMouseDown = (e) => {
    if (!isEditMode) return;
    if (e.target.closest(".widget-content")) return;
    if (e.target.closest(".widget-controls")) return;
    setIsDragging(true);
    setDragStart({
      x: e.clientX - position.x,
      y: e.clientY - position.y,
    });
  };

  useEffect(() => {
    if (!isDragging) return;

    const handleMouseMove = (e) => {
      onMove(widget.id, {
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y,
      });
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    document.addEventListener("mousemove", handleMouseMove);
    document.addEventListener("mouseup", handleMouseUp);

    return () => {
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("mouseup", handleMouseUp);
    };
  }, [isDragging, dragStart, widget.id, onMove]);

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
        </>
      )}
      <div className="widget-content w-full h-full p-8">{children}</div>
    </div>
  );
};

export default DraggableWidget;

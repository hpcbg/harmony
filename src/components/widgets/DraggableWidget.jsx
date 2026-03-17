import { useState, useEffect, useRef } from "react";
import { useDispatch, useSelector } from "react-redux";
import { X, Edit, Menu } from "lucide-react";
import { startEditWidget } from "../../store/widgetsSlice";
import WidgetRenderer from "./WidgetRenderer";
import {
  removeWidgetFromPage,
  moveWidgetOnPage,
  resizeWidget,
  selectPages,
} from "../../store/pagesSlice";
import useConfirm from "../../hooks/useConfirm";
import ConfirmModal from "../modals/ConfirmModal";

export default function DraggableWidget({ widget, position }) {
  const GRID_SIZE = 20;

  const snapToGrid = (value) => Math.floor(value / GRID_SIZE) * GRID_SIZE;

  const dispatch = useDispatch();
  const { currentPage, editMode } = useSelector(selectPages);

  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [containerBounds, setContainerBounds] = useState(null);

  const { confirm, openConfirm, closeConfirm, isOpen } = useConfirm();

  const dragStartRef = useRef({ mouseX: 0, mouseY: 0, widgetX: 0, widgetY: 0 });

  const resizeStartRef = useRef({
    mouseX: 0,
    mouseY: 0,
    widgetX: 0,
    widgetY: 0,
    width: 0,
    height: 0,
  });

  const ref = useRef(null);

  // Get container bounds when component mounts or when dragging/resizing starts
  useEffect(() => {
    const updateContainerBounds = () => {
      if (ref.current && ref.current.parentElement) {
        const parent = ref.current.parentElement;
        const rect = parent.getBoundingClientRect();
        setContainerBounds({
          width: parent.clientWidth,
          height: parent.clientHeight,
          left: rect.left,
          top: rect.top,
        });
      }
    };

    updateContainerBounds();
    window.addEventListener("resize", updateContainerBounds);

    return () => {
      window.removeEventListener("resize", updateContainerBounds);
    };
  }, []);

  const confirmRemoveWidget = (widgetId) => {
    openConfirm({
      type: "remove-widget-from-page",
      widgetId,
      pageId: currentPage,
    });
  };

  const handleConfirm = () => {
    if (confirm.type === "remove-widget-from-page") {
      dispatch(
        removeWidgetFromPage({
          pageId: currentPage,
          widgetId: confirm.widgetId,
        }),
      );
    }

    closeConfirm();
  };
  const handleMouseDown = (e) => {
    if (!editMode) return;
    if (
      e.target.closest(".widget-content") ||
      e.target.closest(".widget-controls") ||
      e.target.closest(".resize-handle")
    )
      return;

    e.preventDefault();
    setIsDragging(true);
    dragStartRef.current = {
      mouseX: snapToGrid(e.clientX),
      mouseY: snapToGrid(e.clientY),
      widgetX: position.x,
      widgetY: position.y,
    };
  };

  const handleResizeMouseDown = (e) => {
    if (!editMode) return;
    e.preventDefault();
    e.stopPropagation();

    setIsResizing(true);
    resizeStartRef.current = {
      mouseX: e.clientX,
      mouseY: e.clientY,
      widgetX: position.x,
      widgetY: position.y,
      width: position.width || 300,
      height: position.height || 200,
    };
  };

  useEffect(() => {
    if (!isDragging && !isResizing) return;

    const handleMouseMove = (e) => {
      if (!containerBounds) return;

      if (isDragging) {
        const deltaX = e.clientX - dragStartRef.current.mouseX;
        const deltaY = e.clientY - dragStartRef.current.mouseY;

        const widgetWidth = position.width || 300;
        const widgetHeight = position.height || 200;

        // Calculate new position
        let newX = dragStartRef.current.widgetX + deltaX;
        let newY = dragStartRef.current.widgetY + deltaY;

        // Constrain to container bounds
        newX = Math.max(0, Math.min(newX, containerBounds.width - widgetWidth));
        newY = Math.max(
          0,
          Math.min(newY, containerBounds.height - widgetHeight),
        );

        dispatch(
          moveWidgetOnPage({
            pageId: currentPage,
            widgetId: widget.id,
            position: {
              x: newX,
              y: newY,
              width: position.width,
              height: position.height,
            },
          }),
        );
      }

      if (isResizing) {
        const deltaX = e.clientX - resizeStartRef.current.mouseX;
        const deltaY = e.clientY - resizeStartRef.current.mouseY;

        // Calculate new dimensions
        let newWidth = Math.max(50, resizeStartRef.current.width + deltaX);
        let newHeight = Math.max(50, resizeStartRef.current.height + deltaY);

        // Constrain width to not exceed container boundary
        const maxWidth = containerBounds.width - resizeStartRef.current.widgetX;
        newWidth = Math.min(newWidth, maxWidth);

        // Constrain height to not exceed container boundary
        const maxHeight =
          containerBounds.height - resizeStartRef.current.widgetY;
        newHeight = Math.min(newHeight, maxHeight);

        if (widget.type == "button") {
          newWidth = Math.max(100, snapToGrid(newWidth));
          newHeight = Math.max(80, snapToGrid(newHeight));
        } else {
          // Apply snap to grid
          newWidth = Math.max(200, snapToGrid(newWidth));
          newHeight = Math.max(100, snapToGrid(newHeight));
        }

        dispatch(
          resizeWidget({
            pageId: currentPage,
            widgetId: widget.id,
            position: {
              x: resizeStartRef.current.widgetX,
              y: resizeStartRef.current.widgetY,
              width: newWidth,
              height: newHeight,
            },
          }),
        );
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
  }, [isDragging, isResizing, dispatch, widget.id, containerBounds]);

  return (
    <div
      ref={ref}
      className={`absolute bg-white rounded-lg shadow-lg ${editMode ? (isDragging ? "cursor-grabbing" : "cursor-grab") : ""
        }`}
      style={{
        left: position.x,
        top: position.y,
        width: position.width || 300,
        height: position.height || 200,
      }}
      onMouseDown={handleMouseDown}
    >
      {editMode && (
        <>
          <div className="absolute top-2 left-2 text-gray-400">
            <Menu size={16} />
          </div>

          <div className="widget-controls absolute top-2 right-2 flex gap-1 z-10">
            <button
              onClick={() => dispatch(startEditWidget(widget.id))}
              className="p-1 text-blue-500 hover:text-blue-700 bg-white rounded"
            >
              <Edit size={16} />
            </button>
            <button
              onClick={() => confirmRemoveWidget(widget.id)}
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

      <ConfirmModal
        isOpen={isOpen}
        message="Are you sure you want to remove the widget from the page?"
        onConfirm={handleConfirm}
        onCancel={closeConfirm}
      />

      <div className="widget-content w-full h-full p-2">
        <WidgetRenderer widget={widget} />
      </div>
    </div>
  );
}

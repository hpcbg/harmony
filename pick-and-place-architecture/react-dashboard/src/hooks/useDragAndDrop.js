import { useState, useEffect, useCallback } from "react";

const useDragAndDrop = () => {
  const [isDragging, setIsDragging] = useState(false);
  const [draggedItemId, setDraggedItemId] = useState(null);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [currentPosition, setCurrentPosition] = useState({ x: 0, y: 0 });

  const startDrag = useCallback(
    (itemId, clientX, clientY, initialX, initialY) => {
      setIsDragging(true);
      setDraggedItemId(itemId);
      setDragStart({
        x: clientX - initialX,
        y: clientY - initialY,
      });
      setCurrentPosition({ x: initialX, y: initialY });
    },
    []
  );

  const updateDragPosition = useCallback(
    (clientX, clientY) => {
      if (!isDragging) return;

      const newX = Math.max(0, clientX - dragStart.x);
      const newY = Math.max(0, clientY - dragStart.y);

      setCurrentPosition({ x: newX, y: newY });
    },
    [isDragging, dragStart]
  );

  const endDrag = useCallback(() => {
    setIsDragging(false);
    setDraggedItemId(null);
  }, []);

  useEffect(() => {
    if (!isDragging) return;

    const handleMouseMove = (e) => {
      updateDragPosition(e.clientX, e.clientY);
    };

    const handleMouseUp = () => {
      endDrag();
    };

    document.addEventListener("mousemove", handleMouseMove);
    document.addEventListener("mouseup", handleMouseUp);

    return () => {
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("mouseup", handleMouseUp);
    };
  }, [isDragging, updateDragPosition, endDrag]);

  return {
    isDragging,
    draggedItemId,
    currentPosition,
    startDrag,
    endDrag,
  };
};

export default useDragAndDrop;

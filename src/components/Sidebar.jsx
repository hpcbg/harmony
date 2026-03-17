import { useDispatch, useSelector } from "react-redux";
import { useState } from "react";
import { LayoutDashboard, Settings, Plus, GripVertical } from "lucide-react";
import {
  setCurrentPage,
  reorderPages,
  startCreatePage,
  selectPages,
} from "../store/pagesSlice";

export default function Sidebar() {
  const dispatch = useDispatch();
  const { pages, currentPage, editMode } = useSelector(selectPages);

  const [draggedPageId, setDraggedPageId] = useState(null);
  const [dragOverPageId, setDragOverPageId] = useState(null);

  const handleAddPage = () => {
    dispatch(startCreatePage());
  };

  // --- Drag & Drop Handlers ---
  const handleDragStart = (e, pageId) => {
    if (!editMode) return;
    setDraggedPageId(pageId);
    e.dataTransfer.effectAllowed = "move";
  };

  const handleDragOver = (e, pageId) => {
    if (!editMode || !draggedPageId) return;
    e.preventDefault();
    e.dataTransfer.dropEffect = "move";
    setDragOverPageId(pageId);
  };

  const handleDragLeave = () => {
    setDragOverPageId(null);
  };

  const handleDrop = (e, targetPageId) => {
    if (!editMode || !draggedPageId) return;
    e.preventDefault();
    if (draggedPageId === targetPageId) {
      setDraggedPageId(null);
      setDragOverPageId(null);
      return;
    }

    const draggedIndex = pages.findIndex((p) => p.id === draggedPageId);
    const targetIndex = pages.findIndex((p) => p.id === targetPageId);

    const newPages = [...pages];
    const [draggedPage] = newPages.splice(draggedIndex, 1);
    newPages.splice(targetIndex, 0, draggedPage);

    dispatch(reorderPages(newPages));
    setDraggedPageId(null);
    setDragOverPageId(null);
  };

  const handleDragEnd = () => {
    setDraggedPageId(null);
    setDragOverPageId(null);
  };

  // --- Click Handlers ---
  const handlePageClick = (pageId) => {
    dispatch(setCurrentPage(pageId));
  };

  const mainPages = pages.filter((page) => page.id !== "settings");
  const settingsPage = pages.find((page) => page.id === "settings");

  return (
    <>
      <div className="w-64 bg-gray-900 text-white flex flex-col">
        {/* Header */}
        <div className="p-4 border-b border-gray-700">
          <h1 className="text-xl font-bold flex items-center gap-2">
            <LayoutDashboard size={24} />
            Dashboard
          </h1>
        </div>

        {/* Pages Navigation */}
        <nav className="flex-1 overflow-y-auto">
          <div className="p-2">
            <div className="text-xs text-gray-400 uppercase px-3 py-2">
              Pages
            </div>
            {mainPages.map((page) => (
              <div
                key={page.id}
                draggable={editMode}
                onDragStart={(e) => handleDragStart(e, page.id)}
                onDragOver={(e) => handleDragOver(e, page.id)}
                onDragLeave={handleDragLeave}
                onDrop={(e) => handleDrop(e, page.id)}
                onDragEnd={handleDragEnd}
                className={`flex items-center gap-2 mb-1 rounded transition-all ${
                  draggedPageId === page.id ? "opacity-50" : ""
                } ${
                  dragOverPageId === page.id ? "border-2 border-blue-400" : ""
                }`}
              >
                {editMode && (
                  <div className="text-gray-500 pl-2 cursor-grab active:cursor-grabbing">
                    <GripVertical size={16} />
                  </div>
                )}
                <button
                  onClick={() => handlePageClick(page.id)}
                  className={`flex-1 text-left px-3 py-2 rounded transition-colors ${
                    currentPage === page.id
                      ? "bg-blue-600"
                      : "hover:bg-gray-800"
                  }`}
                >
                  {page.name}
                </button>
              </div>
            ))}

            {/* Add Page Button (only in edit mode) */}
            {editMode && (
              <button
                onClick={handleAddPage}
                className="w-full text-left px-3 py-2 text-sm text-green-400 
                       hover:text-green-300 hover:bg-gray-800 rounded flex 
                       items-center gap-2 transition-colors"
              >
                <Plus size={16} />
                Add Page
              </button>
            )}
          </div>
        </nav>

        {/* Settings Button */}
        <button
          onClick={() => handlePageClick(settingsPage.id)}
          className={`flex items-center gap-2 p-4 border-t border-gray-700 transition-colors ${
            currentPage === "settings" ? "bg-blue-600" : "hover:bg-gray-800"
          }`}
        >
          <Settings size={20} />
          Settings
        </button>
      </div>
    </>
  );
}

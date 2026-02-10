import { useSelector, useDispatch } from "react-redux";
import { Plus, LayoutDashboard } from "lucide-react";
import { useState } from "react";
import WidgetLibraryModal from "./modals/WidgetLibraryModal";
import { startCreateWidget } from "../store/widgetsSlice";
import { selectPages } from "../store/pagesSlice.js";

export default function Header({ title }) {
  const { currentPage, editMode } = useSelector(selectPages);
  const dispatch = useDispatch();
  const isSettingsPage = currentPage === "settings";
  const [showWidgetLibrary, setShowWidgetLibrary] = useState(false);
  return (
    <div className="bg-white shadow-sm px-6 py-4 flex items-center justify-between">
      <h2 className="text-2xl font-bold text-gray-800">{title}</h2>

      {/* Action Buttons (only show on non-settings pages in edit mode) */}
      {!isSettingsPage && editMode && (
        <div className="flex gap-3">
          <button
            onClick={() => dispatch(startCreateWidget())}
            className="flex items-center gap-2 px-4 py-2 bg-blue-500 
                     text-white rounded-lg hover:bg-blue-600 transition-colors"
          >
            <Plus size={20} />
            New Widget
          </button>
          <button
            onClick={() => setShowWidgetLibrary(true)}
            className="flex items-center gap-2 px-4 py-2 bg-purple-500 
                     text-white rounded-lg hover:bg-purple-600 transition-colors"
          >
            <LayoutDashboard size={20} />
            From Library
          </button>
        </div>
      )}

      {/* Widget Library Modal */}
      {showWidgetLibrary && (
        <WidgetLibraryModal
          isOpen={showWidgetLibrary}
          onClose={() => setShowWidgetLibrary(false)}
        />
      )}
    </div>
  );
}

import { Plus, LayoutDashboard } from "lucide-react";

const Header = ({
  title,
  isSettingsPage,
  isEditMode,
  onAddWidget,
  onShowLibrary,
}) => {
  return (
    <div className="bg-white shadow-sm px-6 py-4 flex items-center justify-between">
      <h2 className="text-2xl font-bold text-gray-800">{title}</h2>

      {/* Action Buttons (only show on non-settings pages in edit mode) */}
      {!isSettingsPage && isEditMode && (
        <div className="flex gap-3">
          <button
            onClick={onAddWidget}
            className="flex items-center gap-2 px-4 py-2 bg-blue-500 
                     text-white rounded-lg hover:bg-blue-600 transition-colors"
          >
            <Plus size={20} />
            New Widget
          </button>
          <button
            onClick={onShowLibrary}
            className="flex items-center gap-2 px-4 py-2 bg-purple-500 
                     text-white rounded-lg hover:bg-purple-600 transition-colors"
          >
            <LayoutDashboard size={20} />
            From Library
          </button>
        </div>
      )}
    </div>
  );
};

export default Header;

import { nanoid } from "@reduxjs/toolkit";
import { useDispatch, useSelector } from "react-redux";
import {
  addPage,
  editPageName,
  selectPages,
  stopEditPage,
} from "../../store/pagesSlice";
import { useState, useEffect } from "react";

export default function PageFormModal() {
  const [name, setName] = useState("");
  const dispatch = useDispatch();

  // Get the editing page from Redux
  const { editingPage, creatingNewPage } = useSelector(selectPages);
  const pageName = editingPage ? editingPage.name : creatingNewPage ? "" : "";

  useEffect(() => {
    if (editingPage) {
      setName(pageName ?? "");
    } else {
      setName(""); // always empty for new page
    }
  }, [editingPage, pageName]);

  const handleSubmit = (e) => {
    e.preventDefault();
    const name = e.target.pageName.value.trim();
    if (!name) return;

    if (editingPage) {
      dispatch(editPageName({ id: editingPage.id, name }));
    } else if (creatingNewPage) {
      dispatch(
        addPage({
          id: nanoid(), // generate id for new page
          name: name,
          widgets: {},
        }),
      );
    }

    dispatch(stopEditPage());
    setName("");
  };

  const handleClose = () => {
    dispatch(stopEditPage());
    setName("");
  };

  // Only render if editing or creating
  if (!editingPage && !creatingNewPage) return null;

  return (
    <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
      <form
        onSubmit={handleSubmit}
        className="bg-white rounded-lg shadow-xl w-full max-w-md p-6"
      >
        <h3 className="text-xl font-bold mb-4">
          {editingPage ? "Edit Page" : "Create New Page"}
        </h3>

        <div className="mb-4">
          <label className="block text-sm font-medium mb-2">Page Name</label>
          <input
            type="text"
            name="pageName"
            defaultValue={pageName}
            onChange={(e) => setName(e.target.value)}
            className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
            placeholder="Enter page name"
          />
        </div>

        <div className="flex gap-3">
          <button
            type="submit"
            disabled={!name.trim()}
            className="flex-1 px-4 py-2 bg-blue-500 text-white rounded-lg 
                       hover:bg-blue-600 disabled:opacity-50 
                       disabled:cursor-not-allowed transition-colors"
          >
            {editingPage ? "Update" : "Create"}
          </button>
          <button
            type="button"
            onClick={handleClose}
            className="flex-1 px-4 py-2 border border-gray-300 rounded-lg hover:bg-gray-50 transition-colors"
          >
            Cancel
          </button>
        </div>
      </form>
    </div>
  );
}

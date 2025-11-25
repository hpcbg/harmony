import React, { useState, useEffect } from 'react';

const PageFormModal = ({ 
  isOpen, 
  onClose, 
  onSubmit, 
  editingPage = null 
}) => {
  const [pageName, setPageName] = useState('');
  
  useEffect(() => {
    if (editingPage) {
      setPageName(editingPage.name);
    } else {
      setPageName('');
    }
  }, [editingPage, isOpen]);
  
  const handleSubmit = () => {
    if (!pageName.trim()) return;
    onSubmit(pageName);
    setPageName('');
  };
  
  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      handleSubmit();
    }
  };
  
  if (!isOpen) return null;
  
  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg shadow-xl w-full max-w-md p-6">
        <h3 className="text-xl font-bold mb-4">
          {editingPage ? 'Edit Page' : 'Create New Page'}
        </h3>
        
        <div className="mb-4">
          <label className="block text-sm font-medium mb-2">
            Page Name
          </label>
          <input
            type="text"
            value={pageName}
            onChange={(e) => setPageName(e.target.value)}
            onKeyPress={handleKeyPress}
            className="w-full px-3 py-2 border rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
            placeholder="Enter page name"
            autoFocus
          />
        </div>
        
        <div className="flex gap-3">
          <button
            onClick={handleSubmit}
            disabled={!pageName.trim()}
            className="flex-1 px-4 py-2 bg-blue-500 text-white rounded-lg 
                     hover:bg-blue-600 disabled:opacity-50 
                     disabled:cursor-not-allowed transition-colors"
          >
            {editingPage ? 'Update' : 'Create'}
          </button>
          <button
            onClick={onClose}
            className="flex-1 px-4 py-2 border rounded-lg hover:bg-gray-50 transition-colors"
          >
            Cancel
          </button>
        </div>
      </div>
    </div>
  );
};

export default PageFormModal;
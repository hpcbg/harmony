const WidgetExistsModal = ({ isOpen, onClose }) => {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 flex items-center justify-center bg-black/40 z-50">
      <div className="bg-white rounded-xl shadow-xl p-6 w-80">
        <h2 className="text-lg font-semibold mb-3 text-gray-800">
          Widget Already Added
        </h2>

        <p className="text-gray-600 mb-6">
          This widget is already used on this page.
        </p>

        <button
          onClick={onClose}
          className="w-full bg-blue-600 text-white py-2 rounded-md hover:bg-blue-700 transition"
        >
          OK
        </button>
      </div>
    </div>
  );
};
export default WidgetExistsModal;

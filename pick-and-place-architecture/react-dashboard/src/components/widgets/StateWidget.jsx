const StateWidget = ({ widget, data }) => (
  <div className="h-full flex flex-col items-center justify-center bg-green-50 rounded p-4">
    <div className="text-large text-gray-600 mb-2">{widget.name}</div>
    <div
      className={`w-16 h-16 rounded-full flex items-center justify-center ${
        data?.value ? "bg-green-500" : "bg-gray-300"
      }`}
    >
      <div className="text-white text-2xl">{data?.value ? "✓" : "✗"}</div>
    </div>
    <div className="text-sm mt-2">{data?.value ? "Active" : "Inactive"}</div>
  </div>
);

export default StateWidget;

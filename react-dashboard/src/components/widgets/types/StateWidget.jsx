import { useSelector } from "react-redux";

export default function StateWidget({ widget }) {
  const widgetData = useSelector((state) => state.widgetData.values);
  const data = widgetData?.[widget.id] ?? { values: [] };

  const isActive = !!data?.value;

  return (
    <div className="h-full flex flex-col items-center justify-center bg-green-50 rounded p-4">
      <div className="text-lg text-gray-600 mb-2">
        {widget.config?.label || widget.name}
      </div>

      <div
        className={`w-16 h-16 rounded-full flex items-center justify-center ${
          isActive ? "bg-green-500" : "bg-gray-300"
        }`}
      >
        <div className="text-white text-2xl">{isActive ? "✓" : "✗"}</div>
      </div>

      <div className="text-sm mt-2">{isActive ? "Active" : "Inactive"}</div>
    </div>
  );
}

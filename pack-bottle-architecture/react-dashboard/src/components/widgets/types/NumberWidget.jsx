import { useSelector } from "react-redux";

export default function NumberWidget({ widget }) {
  const widgetData = useSelector((state) => state.widgetData.values);
  const data = widgetData?.[widget.id] ?? { values: [] };

  return (
    <div className="h-full flex flex-col bg-blue-50 rounded p-4">
      <div className="text-lg text-gray-600 mb-2 flex justify-center items-center">
        {widget.config?.label || widget.name}
      </div>
      <div className="text-base text-blue-600 flex justify-center items-center">
        {data?.value ?? "--"}
      </div>
    </div>
  );
}

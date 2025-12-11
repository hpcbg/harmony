const HistoryWidget = ({ widget, data }) => {
  const values = data?.values || [];
  const max = Math.max(...values, 1);

  return (
    <div className="h-full flex flex-col bg-purple-50 rounded p-4">
      <div className="text-sm text-gray-600 mb-2">{widget.name}</div>
      <div className="flex-1 flex items-end justify-between gap-1">
        {values.map((val, i) => (
          <div
            key={i}
            className="flex-1 bg-purple-500 rounded-t"
            style={{ height: `${(val / max) * 100}%` }}
          />
        ))}
      </div>
    </div>
  );
};

export default HistoryWidget;

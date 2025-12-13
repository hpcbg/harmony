const HistoryWidget = ({ widget, data }) => {
  const values = data?.values || [];
  const max = Math.max(...values, 1);
  const min = Math.min(...values, 0);

  // Calculate axis values
  const range = max - min;
  const step = range > 0 ? Math.ceil(range / 4) : 25;
  const yAxisValues =
    range > 0
      ? [min, min + step, min + step * 2, min + step * 3, max]
      : [0, 25, 50, 75, 100];

  return (
    <div className="h-full flex flex-col bg-purple-50 rounded p-4">
      <div className="text-sm text-gray-600 mb-2 text-center flex justify-center">
        {widget.name}
      </div>
      <div className="flex-1 flex gap-2">
        {/* Y-axis */}
        <div className="flex flex-col justify-between text-xs text-gray-500 w-8 text-right pr-1">
          {yAxisValues
            .slice()
            .reverse()
            .map((val, i) => (
              <div key={i}>{Math.round(val)}</div>
            ))}
        </div>

        {/* Chart area */}
        <div className="flex-1 flex flex-col">
          {/* Bars container */}
          <div className="flex-1 flex items-end justify-between gap-1 border-l border-b border-gray-300 pl-1 pb-1">
            {values.length === 0 ? (
              <div className="flex-1 flex items-center justify-center text-gray-400 text-sm">
                No data available
              </div>
            ) : (
              values.map((val, i) => {
                const height =
                  range > 0 ? ((val - min) / range) * 100 : (val / 100) * 100;

                return (
                  <div
                    key={i}
                    className="flex-1 bg-purple-500 rounded-t transition-all hover:bg-purple-600 relative group"
                    style={{
                      height: `${Math.max(height, 2)}%`,
                      minHeight: "2px",
                    }}
                  >
                    {/* Tooltip on hover */}
                    <div
                      className="absolute bottom-full left-1/2 transform -translate-x-1/2 mb-1 
                                    bg-gray-800 text-white text-xs rounded px-2 py-1 
                                    opacity-0 group-hover:opacity-100 transition-opacity 
                                    pointer-events-none whitespace-nowrap z-10"
                    >
                      {val}
                    </div>
                  </div>
                );
              })
            )}
          </div>

          {/* X-axis */}
          <div className="flex justify-between text-xs text-gray-500 mt-1 pl-1">
            {values.length > 0 && (
              <>
                <div>0</div>
                {values.length > 4 && (
                  <>
                    <div>{Math.floor(values.length / 2)}</div>
                    <div>{values.length - 1}</div>
                  </>
                )}
                {values.length <= 4 && values.length > 1 && (
                  <div>{values.length - 1}</div>
                )}
              </>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default HistoryWidget;

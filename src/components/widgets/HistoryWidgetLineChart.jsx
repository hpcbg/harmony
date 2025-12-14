const HistoryWidgetLineChart = ({ widget, data }) => {
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

  // Generate SVG path for line chart
  const generateLinePath = () => {
    if (values.length === 0) return "";

    const width = 100; // percentage
    const stepX = width / (values.length - 1 || 1);

    let path = "";

    values.forEach((val, i) => {
      const x = i * stepX;
      const y =
        range > 0 ? 100 - ((val - min) / range) * 100 : 100 - (val / 100) * 100;

      if (i === 0) {
        path += `M ${x} ${y}`;
      } else {
        path += ` L ${x} ${y}`;
      }
    });

    return path;
  };

  // Generate points for dots
  const generatePoints = () => {
    if (values.length === 0) return [];

    const width = 100;
    const stepX = width / (values.length - 1 || 1);

    return values.map((val, i) => {
      const x = i * stepX;
      const y =
        range > 0 ? 100 - ((val - min) / range) * 100 : 100 - (val / 100) * 100;

      return { x, y, value: val, index: i };
    });
  };

  const linePath = generateLinePath();
  const points = generatePoints();

  return (
    <div className="h-full flex flex-col bg-purple-50 rounded p-4">
      <div className="text-large text-gray-600 mb-2 text-center flex justify-center">
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
          {/* Line chart container */}
          <div className="flex-1 border-l border-b border-gray-300 relative">
            {values.length === 0 ? (
              <div className="absolute inset-0 flex items-center justify-center text-gray-400 text-sm">
                No data available
              </div>
            ) : (
              <div>
                <svg
                  className="absolute inset-0 w-full h-full"
                  viewBox="0 0 100 100"
                  preserveAspectRatio="none"
                  style={{ overflow: "visible" }}
                >
                  {/* Grid lines */}
                  <g opacity="0.1">
                    {[0, 25, 50, 75, 100].map((y) => (
                      <line
                        key={y}
                        x1="0"
                        y1={y}
                        x2="100"
                        y2={y}
                        stroke="currentColor"
                        strokeWidth="0.5"
                        vectorEffect="non-scaling-stroke"
                      />
                    ))}
                  </g>

                  {/* Area under the line (gradient fill) */}
                  <defs>
                    <linearGradient
                      id={`gradient-${widget.id}`}
                      x1="0%"
                      y1="0%"
                      x2="0%"
                      y2="100%"
                    >
                      <stop offset="0%" stopColor="#9333ea" stopOpacity="0.3" />
                      <stop
                        offset="100%"
                        stopColor="#9333ea"
                        stopOpacity="0.05"
                      />
                    </linearGradient>
                  </defs>

                  {linePath && (
                    <>
                      {/* Filled area under line */}
                      <path
                        d={`${linePath} L ${100} 100 L 0 100 Z`}
                        fill={`url(#gradient-${widget.id})`}
                      />

                      {/* Line */}
                      <path
                        d={linePath}
                        fill="none"
                        stroke="#9333ea"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        vectorEffect="non-scaling-stroke"
                      />
                    </>
                  )}
                </svg>

                <div className="absolute inset-0 w-full h-full">
                  {points.map((point, i) => (
                    <div
                      key={i}
                      className="absolute group"
                      style={{
                        left: `${point.x}%`,
                        top: `${point.y}%`,
                        transform: "translate(-50%, -50%)",
                      }}
                    >
                      {/* Dot */}
                      <div
                        className="w-2 h-2 bg-purple-600 rounded-full border-2 border-white 
                                    group-hover:w-3 group-hover:h-3 transition-all"
                      />

                      {/* Tooltip */}
                      <div
                        className="absolute bottom-full left-1/2 transform -translate-x-1/2 mb-2 
                                    bg-gray-800 text-white text-xs rounded px-2 py-1 
                                    opacity-0 group-hover:opacity-100 transition-opacity 
                                    pointer-events-none whitespace-nowrap z-10"
                      >
                        {point.value}
                      </div>
                    </div>
                  ))}
                </div>
              </div>
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

export default HistoryWidgetLineChart;

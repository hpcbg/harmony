const NumberWidget = ({ widget, data }) => (
  <div className="h-full flex flex-col bg-blue-50 rounded p-4">
    <div className="text-large text-gray-600 mb-2 items-center flex justify-center">
      {widget.name}
    </div>
    <div className="text-base text-blue-600 items-center flex justify-center">
      {data?.value ?? "--"}
    </div>
  </div>
);

export default NumberWidget;

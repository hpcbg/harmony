import React from 'react';

const NumberWidget = ({ widget, data }) => (
  <div className="h-full flex flex-col items-center justify-center bg-blue-50 rounded p-4">
    <div className="text-sm text-gray-600 mb-2">{widget.name}</div>
    <div className="text-4xl font-bold text-blue-600">{data?.value ?? '--'}</div>
  </div>
);

export default NumberWidget;
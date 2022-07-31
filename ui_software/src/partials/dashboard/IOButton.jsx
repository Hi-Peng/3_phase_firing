import React from 'react';
import BarChart from '../../charts/BarChart03';
import { CircleSlider } from "react-circle-slider";

// Import utilities
import { tailwindConfig, hexToRGB } from '../../utils/Utils';

function IOButton() {



  return (
    <div className="col-span-full sm:col-span-6 xl:col-span-4 bg-white shadow-lg rounded-sm border border-slate-200">
      <header className="px-5 py-4 border-b border-slate-100">
        <h2 className="font-semibold text-slate-800">I/O Control</h2>
      </header>

      <div className="grow">
        {/* Change the height attribute to adjust the chart height */}
        <CircleSlider
        size = {389}
            knobRadius={15}
            progressWidth={20}
            max = {180}
            min = {0}
            circleWidth={3}
            progressColor="#6656B6"
            tooltipColor="#6ab6e1"
            showTooltip={true}
            tooltipSize={26}
        />
      </div>
    </div>
  );
}

export default IOButton;

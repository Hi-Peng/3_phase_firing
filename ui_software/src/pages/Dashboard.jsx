import React, { useState } from 'react';

import WelcomeBanner from '../partials/dashboard/WelcomeBanner';
import TimingPlot from '../partials/dashboard/TimingPlot';
import IOButton from '../partials/dashboard/IOButton';
import AngleKnob
 from '../partials/dashboard/AngleKnob';
import Banner from '../partials/Banner';

function Dashboard() {

  const [sidebarOpen, setSidebarOpen] = useState(false);

  return (
    <div className="flex h-screen overflow-hidden">
      {/* Content area */}
      <div className="relative flex flex-col flex-1 overflow-y-auto overflow-x-hidden">
        <main>
          <div className="px-4 sm:px-6 lg:px-8 py-8 w-full max-w-9xl mx-auto">

            {/* Welcome banner */}
            <WelcomeBanner />

            {/* Cards */}
            <div className="grid grid-cols-3 gap-4">
              <TimingPlot />          
              <IOButton /> 
              <AngleKnob />
            </div>

          </div>
        </main>

        <Banner />

      </div>
    </div>
  );
}

export default Dashboard;
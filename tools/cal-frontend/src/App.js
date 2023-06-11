import logo from './logo.svg';
import './App.css';
import Backend from './Backend';
import { ParamList, ParamHeader } from './Param/Param';
import ControlHeader from './ControlHeader/ControlHeader';
import DebugConsole from './DebugConsole/DebugConsole';
import { useState } from 'react';

function App() {

  const [params, setparams] = useState([]);
  Backend.setParams = setparams;


  return (
    <div className="App">

      <ControlHeader />

      <div>
        <table className='param-table'>
          <ParamHeader />
          <ParamList params={params}/>
        </table>
      </div>

      <DebugConsole />

    </div>
  );
}

export default App;

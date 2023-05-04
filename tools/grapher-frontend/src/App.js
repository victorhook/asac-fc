import './App.css';

import Tabs from './Tabs';


function App() {
  return (
    <div className="container-fluid">
      <nav className="navbar navbar-expand-lg navbar-light bg-light">
        <button className="btn m-1">Connect</button>
        <button className="btn m-1">Disconnect</button>
        <button className="btn m-1">Save</button>
      </nav>

      <div class="row">
        <div class="col-12">
          <Tabs />
        </div>
      </div>

    </div>
  );
}

export default App;

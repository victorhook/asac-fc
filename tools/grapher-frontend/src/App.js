import './App.css';

import Tabs from './Tabs';


function App() {
  return (
    <div className="container-fluid">
      <nav className="navbar navbar-expand-lg navbar-light bg-light">
        <a className="nav-brand nav-link fs-2 m-1 me-5" href="/">
          ASAC Plotter
        </a>
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

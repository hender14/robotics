use super::super::debug::plot;

fn slam() {
    let array = plot::read();
    let dim = array.0.len().pow(3);

    for n in 1..10000 {
        // エッジ、大きな精度行列、係数ベクトルの作成##
        let edges = make_edges(array.0, array.1);  //返す変数が2つになるので「_」で合わせる

    //     for i in range(len(hat_xs)-1): #行動エッジの追加
    //         edges.append(MotionEdge(i, i+1, hat_xs, us, delta))
            
    //     Omega = np.zeros((dim, dim))
    //     xi = np.zeros(dim)
    //     Omega[0:3, 0:3] += np.eye(3)*1000000

    //     ##軌跡を動かす量（差分）の計算##
    //     for e in edges:
    //         add_edge(e, Omega, xi) 

    //     delta_xs = np.linalg.inv(Omega).dot(xi) 
        
    //     ##推定値の更新##
    //     for i in range(len(hat_xs)):
    //         hat_xs[i] += delta_xs[i*3:(i+1)*3] 
            
    //     ##終了判定##
    //     diff = np.linalg.norm(delta_xs) 
    //     print("{}回目の繰り返し: {}".format(n, diff))
    //     if diff < 0.02:
    //         draw(hat_xs, zlist, edges)
    //         break
    }
}

def make_edges(hat_xs, zlist):
    edges = []
    for i in range(len(hat_xs)):
        for j in range(len(zlist[i][1])//3):
            edges.append(ObserveEdge(i, zlist[i][1][j*3:(j+1)*3], zlist[i][0], hat_xs))
    return edges
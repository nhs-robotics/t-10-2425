package com.pocolifo.robobase.control.input.nodes;


import androidx.annotation.NonNull;

import com.pocolifo.robobase.control.input.InputManager;
import com.pocolifo.robobase.control.input.InputManagerNodeResult;
import com.pocolifo.robobase.control.input.nodeUtils.MathUtils;

public class MultiInputNode extends InputManagerInputNode {
    private final InputManagerInputNode[] childs;

    private final InputManagerNodeResult result = new InputManagerNodeResult();

    public MultiInputNode(InputManagerInputNode... childs) {
        this.childs = childs;
    }

    @Override
    public void init(InputManager boss) {
        for(InputManagerInputNode node : childs) node.init(boss);
    }

    public void update() {
        for(InputManagerInputNode n : childs) n.update();
    }

    @NonNull
    @Override
    public InputManagerNodeResult getResult() {
        InputManagerNodeResult[] vals = new InputManagerNodeResult[childs.length];

        for(int i = 0; i < vals.length; i++) vals[i] = childs[i].getResult();

        result.setChildren(vals);
        return result;
    }

    @Override
    public int complexity() {
        int r = 0;
        for(InputManagerInputNode n : childs) r += n.complexity();
        return r + 1;
    }

    @Override
    public String[] getKeysUsed() {
        String[][] keylists = new String[childs.length][];
        for(int i = 0; i < childs.length; i++) {
            keylists[i] = childs[i].getKeysUsed();
        }
        return MathUtils.concatArrays(keylists);
    }

    @Override
    public boolean usesKey(String s) {
        for(InputManagerInputNode n : childs) {
            if(n.usesKey(s)) return true;
        }
        return false;
    }
}
